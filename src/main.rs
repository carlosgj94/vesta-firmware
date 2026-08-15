#![no_std]
#![no_main]

use bme68x::blocking::Bme68x;
use bme68x::interface::{I2cAddress, I2cInterface};
use bme68x::{
    Configuration, Error as Bme68xError, Filter, HeaterConfiguration, OperationMode, Oversampling,
    StandbyTime,
};
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::pac;
use embassy_stm32::time::Hertz;
use embassy_time::{Delay, Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

const HEATER_TEMPERATURE_CELSIUS: u16 = 300;
const HEATER_DURATION_MS: u16 = 100;
const MEASUREMENT_MARGIN_US: u64 = 5_000;
const SAMPLE_INTERVAL: Duration = Duration::from_secs(5);

#[embassy_executor::main(
    executor = "embassy_stm32::executor::Executor",
    entry = "cortex_m_rt::entry"
)]

async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();

    config.enable_debug_during_sleep = cfg!(feature = "debug-sleep");
    // Only pay the STOP entry/exit overhead when the next wake-up is at least
    // this far away. It must be shorter than our five-second sample interval so
    // the RTC wake alarm can be armed before a real STOP-mode build sleeps.
    config.min_stop_pause = Duration::from_millis(250);

    let peripherals = embassy_stm32::init(config);

    // RAK3172 board wiring: BME688 SCL=PA12 and SDA=PA11 on I2C2. The PCB has
    // external 10 kohm pull-ups, validated by the earlier C bring-up. Stay at
    // 100 kHz for maximum margin.
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz::khz(100);
    i2c_config.scl_pullup = false;
    i2c_config.sda_pullup = false;

    let mut i2c = I2c::new_blocking(
        peripherals.I2C2,
        peripherals.PA12,
        peripherals.PA11,
        i2c_config,
    );

    // Allow the always-powered sensor to finish its cold-start sequence before
    // the diagnostic chip-ID transaction. This also makes a VOUT power cycle
    // deterministic when reset and target power rise at nearly the same time.
    Timer::after(Duration::from_millis(10)).await;

    // Keep the peripheral and pins alive if the first transaction fails so the
    // diagnostic reports the actual bus levels instead of the pins after Drop.
    let mut preflight_chip_id = [0_u8];
    let (sensor_address, address_failures) =
        match i2c.blocking_write_read(0x76_u8, &[0xd0], &mut preflight_chip_id) {
            Ok(()) => (I2cAddress::Low, None),
            Err(low_error) => {
                let mut high_address_chip_id = [0_u8];
                match i2c.blocking_write_read(0x77_u8, &[0xd0], &mut high_address_chip_id) {
                    Ok(()) => {
                        preflight_chip_id = high_address_chip_id;
                        warn!("BME68x answered at diagnostic address 0x77, not wired address 0x76");
                        (I2cAddress::High, None)
                    }
                    Err(high_error) => (I2cAddress::Low, Some((low_error, high_error))),
                }
            }
        };

    if let Some((low_error, high_error)) = address_failures {
        let gpio_inputs = pac::GPIOA.idr().read();
        let i2c_status = pac::I2C2.isr().read();
        error!(
            "BME688 I2C preflight failed: error_0x76={:?}, error_0x77={:?}, SCL(PA12)={:?}, SDA(PA11)={:?}, I2C2_ISR={:?}",
            low_error,
            high_error,
            gpio_inputs.idr(12),
            gpio_inputs.idr(11),
            i2c_status
        );

        // A master can hold both lines low while a timed-out transfer is still
        // active. Disable I2C2 briefly so its open-drain outputs release the
        // bus, then report the idle levels supplied by the PCB pull-ups.
        pac::I2C2.cr1().modify(|control| control.set_pe(false));
        Timer::after(Duration::from_millis(1)).await;
        let released_inputs = pac::GPIOA.idr().read();
        error!(
            "BME688 released bus levels: SCL(PA12)={:?}, SDA(PA11)={:?}",
            released_inputs.idr(12),
            released_inputs.idr(11)
        );

        // This never returns, deliberately retaining ownership of `i2c` and
        // leaving PA12/PA11 in alternate-function mode for electrical probing.
        // I2C2 itself remains disabled, so no more bus traffic is generated.
        loop {
            let _keep_i2c_configured = &i2c;
            Timer::after(SAMPLE_INTERVAL).await;
            error!("BME688 unavailable; check the reported SCL/SDA levels and reset the board");
        }
    }
    let sensor_address_byte = u8::from(sensor_address);
    info!(
        "BME688 I2C preflight: address=0x{:02x}, register 0xd0 returned 0x{:02x}",
        sensor_address_byte, preflight_chip_id[0]
    );

    let interface = I2cInterface::new(i2c, sensor_address);

    let mut sensor = match Bme68x::new(interface, Delay) {
        Ok(sensor) => sensor,
        Err(sensor_error) => {
            log_sensor_error("initialization", &sensor_error);
            wait_for_reset().await;
        }
    };

    info!(
        "BME68x initialized: chip_id=0x{:02x}, variant={:?}, address=0x{:02x}, I2C2=100kHz",
        sensor.chip_id(),
        sensor.variant(),
        sensor_address_byte
    );

    // Match Bosch's official forced-mode example: temperature x2, pressure
    // x1, humidity x16, filter off, and a 300 C heater for 100 ms.
    let sensor_configuration = Configuration {
        humidity_oversampling: Oversampling::X16,
        temperature_oversampling: Oversampling::X2,
        pressure_oversampling: Oversampling::X1,
        filter: Filter::Off,
        standby_time: StandbyTime::None,
    };
    let heater_configuration = HeaterConfiguration::Forced {
        enabled: true,
        temperature_celsius: HEATER_TEMPERATURE_CELSIUS,
        duration_ms: HEATER_DURATION_MS,
    };

    if let Err(sensor_error) = sensor.set_configuration(&sensor_configuration) {
        log_sensor_error("configuration", &sensor_error);
        wait_for_reset().await;
    }
    if let Err(sensor_error) = sensor.set_heater_configuration(&heater_configuration) {
        log_sensor_error("heater configuration", &sensor_error);
        wait_for_reset().await;
    }

    let conversion_wait = Duration::from_micros(
        u64::from(bme68x::compensation::measurement_duration_us(
            OperationMode::Forced,
            &sensor_configuration,
        )) + u64::from(HEATER_DURATION_MS) * 1_000
            + MEASUREMENT_MARGIN_US,
    );

    info!("Vesta Rust Embassy firmware started");
    info!(
        "BME688 forced-mode loop: heater={} C for {} ms, conversion wait={} us",
        HEATER_TEMPERATURE_CELSIUS,
        HEATER_DURATION_MS,
        conversion_wait.as_micros()
    );

    loop {
        match sensor.set_operation_mode(OperationMode::Forced) {
            Ok(()) => {
                Timer::after(conversion_wait).await;

                match sensor.measurements(OperationMode::Forced) {
                    Ok(measurements) if measurements.is_empty() => {
                        warn!("BME688 conversion completed but no new data field was available");
                    }
                    Ok(measurements) => {
                        for measurement in &measurements {
                            let values = measurement.values;
                            info!(
                                "BME688: temperature_centi_c={}, pressure_pa={}, humidity_milli_percent={}, gas_ohms={}, status=0x{:02x}, new={}, gas_valid={}, heater_stable={}",
                                values.temperature,
                                values.pressure,
                                values.humidity,
                                values.gas_resistance,
                                measurement.status.bits(),
                                measurement.status.is_new(),
                                measurement.status.gas_valid(),
                                measurement.status.heater_stable()
                            );
                        }
                    }
                    Err(sensor_error) => log_sensor_error("data read", &sensor_error),
                }
            }
            Err(sensor_error) => log_sensor_error("forced-mode trigger", &sensor_error),
        }

        // Forced mode returns the BME688 to sleep automatically. Embassy can
        // put the MCU into STOP during this interval; no radio peripheral has
        // been initialized anywhere in this test firmware.
        Timer::after(SAMPLE_INTERVAL).await;
    }
}

async fn wait_for_reset() -> ! {
    loop {
        Timer::after(SAMPLE_INTERVAL).await;
        error!("BME688 unavailable; reset the board after checking power and I2C wiring");
    }
}

fn log_sensor_error(operation: &str, sensor_error: &Bme68xError<i2c::Error>) {
    match sensor_error {
        Bme68xError::Bus(bus_error) => {
            error!("BME688 {} failed: I2C error {:?}", operation, bus_error);
        }
        Bme68xError::UnexpectedChipId { found } => {
            error!(
                "BME688 {} failed: expected chip ID 0x61, read 0x{:02x}",
                operation, found
            );
        }
        Bme68xError::InvalidRegisterValue { register, value } => {
            error!(
                "BME688 {} failed: invalid value 0x{:02x} in register 0x{:02x}",
                operation, value, register
            );
        }
        Bme68xError::InvalidConfiguration(configuration_error) => {
            error!(
                "BME688 {} failed: invalid configuration {:?}",
                operation, configuration_error
            );
        }
        Bme68xError::SelfTestFailed(reason) => {
            error!("BME688 {} failed: self-test {:?}", operation, reason);
        }
        Bme68xError::Timeout => {
            error!("BME688 {} failed: sensor timeout", operation);
        }
    }
}
