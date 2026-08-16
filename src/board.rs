//! RAK3172 board initialization and BME688 bus diagnostics.

use bme68x::interface::I2cAddress;
use defmt::{error, warn};
use embassy_stm32::i2c::{self, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};

const SENSOR_STARTUP_DELAY: Duration = Duration::from_millis(10);
const FAILURE_REPORT_INTERVAL: Duration = Duration::from_secs(5);

/// The concrete I2C bus wired to the BME688 on this board.
pub type SensorBus = I2c<'static, Blocking, Master>;

/// Initialized board resources owned by the application.
pub struct Board {
    sensor_bus: SensorBus,
}

impl Board {
    /// Transfer ownership of the BME688 bus to the sensor workflow.
    pub fn into_sensor_bus(self) -> SensorBus {
        self.sensor_bus
    }
}

/// A responsive BME68x and the bus on which it was found.
pub struct SensorProbe {
    bus: SensorBus,
    address: I2cAddress,
    chip_id: u8,
}

/// Failed address probes together with ownership of the still-configured bus.
pub struct ProbeFailure {
    bus: SensorBus,
    low_error: i2c::Error,
    high_error: i2c::Error,
}

impl ProbeFailure {
    pub(crate) fn into_parts(self) -> (SensorBus, i2c::Error, i2c::Error) {
        (self.bus, self.low_error, self.high_error)
    }
}

impl SensorProbe {
    pub fn address_byte(&self) -> u8 {
        self.address.into()
    }

    pub const fn chip_id(&self) -> u8 {
        self.chip_id
    }

    pub(crate) fn into_parts(self) -> (SensorBus, I2cAddress) {
        (self.bus, self.address)
    }
}

/// Initialize the STM32WLE5 and the board's sensor bus.
pub fn init() -> Board {
    let mut config = embassy_stm32::Config::default();

    // Real STOP mode is the production default. The opt-in debug feature keeps
    // SWD/RTT available by substituting shallow sleep for STOP.
    config.enable_debug_during_sleep = cfg!(feature = "debug-sleep");
    // Only pay the STOP entry/exit overhead when the next wake-up is at least
    // this far away. It remains shorter than the five-second sample interval.
    config.min_stop_pause = Duration::from_millis(250);

    let peripherals = embassy_stm32::init(config);

    // RAK3172 board wiring: BME688 SCL=PA12 and SDA=PA11 on I2C2. The PCB has
    // external 10 kohm pull-ups, validated by the earlier C bring-up.
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz::khz(100);
    i2c_config.scl_pullup = false;
    i2c_config.sda_pullup = false;

    let sensor_bus = I2c::new_blocking(
        peripherals.I2C2,
        peripherals.PA12,
        peripherals.PA11,
        i2c_config,
    );

    Board { sensor_bus }
}

/// Find the BME688 while preserving the bus on failure for diagnostics/recovery.
pub async fn probe_bme688(mut bus: SensorBus) -> Result<SensorProbe, ProbeFailure> {
    // Make a VOUT power cycle deterministic when reset and target power rise at
    // nearly the same time.
    Timer::after(SENSOR_STARTUP_DELAY).await;

    let mut chip_id = [0_u8];
    let address = match bus.blocking_write_read(0x76_u8, &[0xd0], &mut chip_id) {
        Ok(()) => I2cAddress::Low,
        Err(low_error) => {
            let mut high_address_chip_id = [0_u8];
            match bus.blocking_write_read(0x77_u8, &[0xd0], &mut high_address_chip_id) {
                Ok(()) => {
                    chip_id = high_address_chip_id;
                    warn!("BME68x answered at diagnostic address 0x77, not wired address 0x76");
                    I2cAddress::High
                }
                Err(high_error) => {
                    return Err(ProbeFailure {
                        bus,
                        low_error,
                        high_error,
                    });
                }
            }
        }
    };

    Ok(SensorProbe {
        bus,
        address,
        chip_id: chip_id[0],
    })
}

/// Report a failed probe, release the bus lines, and retain the I2C resources.
pub async fn halt_after_probe_failure(failure: ProbeFailure) -> ! {
    let (bus, low_error, high_error) = failure.into_parts();
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

    // Release both open-drain lines after a failed transfer, then report the
    // idle levels supplied by the board's external pull-ups.
    pac::I2C2.cr1().modify(|control| control.set_pe(false));
    Timer::after(Duration::from_millis(1)).await;
    let released_inputs = pac::GPIOA.idr().read();
    error!(
        "BME688 released bus levels: SCL(PA12)={:?}, SDA(PA11)={:?}",
        released_inputs.idr(12),
        released_inputs.idr(11)
    );

    // Retain ownership of the I2C driver and pins. I2C2 remains disabled, so
    // this state is safe for electrical probing and emits no more bus traffic.
    loop {
        let _keep_bus_and_pins_alive = &bus;
        Timer::after(FAILURE_REPORT_INTERVAL).await;
        error!("BME688 unavailable; check the reported SCL/SDA levels and reset the board");
    }
}
