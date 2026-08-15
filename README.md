# Vesta RAK3172 bring-up firmware

This is a temporary Rust/Embassy hardware-validation project for the fabricated
RAK3172 board, whose module contains an STM32WLE5CC. It does not initialize the
STM32 SubGHz radio and cannot transmit RF.

The sensor driver is the published
[`bme68x` 0.1.0 crate](https://crates.io/crates/bme68x), used with its `blocking`
and `defmt` features and without default features.

## Current result (2026-08-15)

- ST-LINK/SWD, flash programming, RTT, and Cortex-M4 execution work.
- The assembled board was previously validated with the Bosch C driver: the
  BME688 returned chip ID `0x61` and valid temperature, pressure, humidity, and
  gas readings.
- The BME688 bus is I2C2 on PA12/SCL and PA11/SDA at 100 kHz, address `0x76`,
  using the PCB's external 10 kohm pull-ups.
- The Rust firmware selects the single-core STM32WLE5CC target and uses that same
  verified bus mapping.
- The corrected Rust image was flashed and verified successfully. It returned
  chip ID `0x61`, variant `GasHigh`, and repeated valid measurements with the
  heater-stable flag set after the first warm-up sample.

One captured stable sample was 24.90 C, 1013.85 hPa, 39.094% RH, and 6200 ohm
gas resistance. Gas resistance then rose normally across subsequent heater
cycles while all samples remained gas-valid and heater-stable.

The firmware performs a chip-ID preflight before initializing the reusable
driver. On failure it disables I2C2, reports the live line/status state, and
generates no further bus traffic.

## Build and run

```bash
cargo build --release
cargo run --release
```

The default `debug-sleep` feature keeps SWD/RTT usable. A power-oriented build is:

```bash
cargo build --release --no-default-features
```

The no-default-features build allows Embassy to use deeper low-power modes during
the five-second interval. Actual sleep current remains a separate hardware
acceptance test.

## Verified hardware mapping

- Module MCU: STM32WLE5CCU6
- I2C peripheral: I2C2
- SCL: PA12, AF4
- SDA: PA11, AF4
- BME688 address: `0x76`
- Pull-ups: external 10 kohm resistors on the PCB

PB6/PB7 and I2C1 belong to an obsolete standalone-MCU revision and are not the
BME688 connection on this assembled RAK3172 board.

## Embassy revision

The project pins Embassy to commit
`3e99135a9e1bdfae06f5c9e88e9af96c8886d2eb`. The crates.io 0.6.0 source enables
STM32WL flash prefetch before lowering the flash latency and reproducibly
hard-faults during startup on this board. The pinned upstream commit changes the
order to set and confirm latency before enabling prefetch.
