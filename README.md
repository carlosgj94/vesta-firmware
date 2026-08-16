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
driver. On failure it reports both address errors and the physical SCL/SDA
levels, then generates no further bus traffic.

## Build and run

```bash
cargo build --release
cargo run --release
```

The default build permits real STOP-mode sleep. For a development session that
keeps SWD/RTT continuously available, opt into debugger-preserving shallow
sleep:

```bash
cargo run --release --features debug-sleep
```

The reusable sensor driver retains its calibration and configuration, but its
board bus creates the concrete I2C2 driver only for the duration of one
transaction attempt. Dropping that temporary driver disables I2C2 and releases
its pins, so the default build is eligible for STOP2 during the one-minute
interval. Actual STOP2 entry and sleep current remain a separate hardware
acceptance test.

The one-minute interval is an explicit bring-up placeholder in `src/main.rs`.
The periodic ticker anchors wake-ups to fixed deadlines, so measurement and
future radio-transmission time do not accumulate into the sampling cadence.

RTT is configured as non-blocking, so disconnecting the debugger cannot stall
the sensing loop. Diagnostic frames may be dropped if the host does not consume
them quickly enough.

## Source layout

- `src/board.rs` owns STM32WLE5 initialization, the exact RAK3172 I2C wiring,
  transaction-scoped I2C ownership, address preflight, and electrical failure
  diagnostics.
- `src/bme688.rs` wraps the generic driver with this application's forced-mode,
  oversampling, heater, and conversion-timing policy.
- `src/output.rs` is the replaceable measurement-output boundary; it writes RTT
  today and will later own LoRa transmission.
- `src/diagnostics.rs` owns startup and error reporting; board-level electrical
  diagnostics remain in `board.rs`.
- `src/main.rs` only orchestrates startup and the periodic sampling cadence.

This separation keeps board details out of the measurement workflow and keeps
the reusable cross-platform abstraction in the published `bme68x` crate.

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
