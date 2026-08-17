# Vesta RAK3172 sensor and LoRa firmware

Rust/Embassy firmware for the fabricated Vesta board. Its
`RAK3172-T-8-SM-I` module contains an STM32WLE5CC, a 3.0 V TCXO, an EU868
radio front end, and an IPEX antenna connector.

Every cycle the firmware:

1. wakes and triggers one forced BME688 conversion;
2. reads compensated values, raw ADC channels, and field/heater metadata;
3. encodes each returned field as a versioned 48-byte frame;
4. transmits the frame over raw LoRa P2P; and
5. cold-sleeps the radio, releases I2C2 and SUBGHZSPI, then waits for the next
   one-minute deadline in STOP2-eligible idle.

The sensor driver is the published
[`bme68x` 0.1.0 crate](https://crates.io/crates/bme68x). The radio physical
layer uses [`lora-phy` 3.0.1](https://crates.io/crates/lora-phy).

## Validation status (2026-08-17)

- ST-LINK/SWD, flash programming, RTT, and Cortex-M4 execution work.
- The BME688 has been read successfully from this assembled board at address
  `0x76`; repeated gas-valid and heater-stable measurements were captured.
- The LoRa firmware passes locked production and `debug-sleep` release builds,
  strict Clippy, and exact host-side wire-format tests.
- The transmitter image has **not** yet been flashed or allowed to transmit.
- No packet has yet been received by the Raspberry Pi HAT.
- Source ownership makes the production idle structurally eligible for STOP2,
  but actual STOP2 entry and current still require a hardware power trace.

## Build and run

```bash
cargo build --release --locked
```

Running the image starts transmitting immediately after the first sensor
sample. Never run it without the correct 868 MHz antenna fully attached to the
RAK3172 module's IPEX connector.

```bash
cargo run --release --locked
```

The default build permits real STOP-mode sleep. For a development session that
keeps SWD/RTT continuously available, opt into debugger-preserving shallow
sleep:

```bash
cargo run --release --locked --features debug-sleep
```

`debug-sleep` deliberately does not demonstrate STOP2 or production current.
RTT is non-blocking, so disconnecting the debugger cannot stall the sensing and
transmit loop.

## Radio configuration

The first bring-up uses raw LoRa P2P rather than LoRaWAN:

- frequency: 868.100 MHz
- bandwidth: 125 kHz
- spreading factor: SF7
- coding rate: 4/5
- preamble: 8 symbols
- header: explicit
- PHY CRC: enabled
- IQ: normal
- private sync word: `0x1424`
- transmit power: +5 dBm

RAK3172-T configuration is board-specific: the firmware selects the STM32WL
high-power PA, enables DCDC, supplies the TCXO through radio DIO3 at 3.0 V, and
uses PB8 for RX switching and PC13 for TX switching. Both switch controls are
forced low on every exit path. Busy and TX-completion waits are bounded so a
radio fault cannot keep the MCU awake indefinitely. Session teardown also holds
the SubGHz core in reset, matching ST's deinitialization fallback and ensuring
an initialization or sleep error cannot leave the radio or TCXO awake.

Raw P2P is suitable for bring-up, but it provides neither encryption nor
authentication. Production wildfire alerts will need LoRaWAN or an
application-level authenticated protocol.

## Telemetry frame v1

Every frame is exactly 48 bytes and uses big-endian integers. The equivalent
Python `struct` format is `>2sBBQIhIIIIIHH6B`.

| Offset | Size | Field | Unit/meaning |
| ---: | ---: | --- | --- |
| 0 | 2 | magic | ASCII `VS` |
| 2 | 1 | version | `1` |
| 3 | 1 | status | BME68x field-status bits |
| 4 | 8 | node ID | FNV-1a of the 96-bit STM32 UID; identifier, not a secret |
| 12 | 4 | sequence | wraps at `u32::MAX`; restarts after reset |
| 16 | 2 | temperature | signed centi-degrees Celsius |
| 18 | 4 | pressure | pascals |
| 22 | 4 | humidity | milli-percent relative humidity |
| 26 | 4 | gas resistance | ohms |
| 30 | 4 | raw temperature | ADC code |
| 34 | 4 | raw pressure | ADC code |
| 38 | 2 | raw humidity | ADC code |
| 40 | 2 | raw gas | ADC code |
| 42 | 1 | gas range | Bosch range code |
| 43 | 1 | gas index | heater-profile index |
| 44 | 1 | measurement index | sensor field counter |
| 45 | 1 | heater resistance | raw register value |
| 46 | 1 | heater current | raw register value |
| 47 | 1 | gas wait | raw register value |

The host fixture in `src/payload.rs` pins every offset and signed/unsigned byte
order before the Raspberry Pi decoder is introduced.

## Power and cadence

The first sample happens immediately at boot. Later sample starts are anchored
to fixed one-minute deadlines, so sensor and radio work do not accumulate into
schedule drift. The interval remains a bring-up placeholder in `src/main.rs`.

The reusable BME688 driver retains calibration and configuration, while its
board bus constructs the concrete I2C2 driver only for a transaction attempt.
Likewise, the radio owns only peripheral tokens while idle and constructs the
concrete SUBGHZSPI/DMA/GPIO drivers for one measurement batch. After cold radio
sleep, dropping those drivers removes their RCC constraints before the long
timer wait.

The timer currently uses the STM32's internal LSI, so a nominal minute can
drift with oscillator tolerance and temperature. Deployment timing and sleep
current must be measured on hardware.

## Source layout

- `src/board.rs`: MCU clocks, exact board wiring, owned sensor/radio resources,
  BME688 address preflight, and electrical failure diagnostics.
- `src/bme688.rs`: application forced-mode, oversampling, heater, and conversion
  policy over the generic published driver.
- `src/payload.rs`: target-independent telemetry v1 encoder and byte fixtures.
- `src/radio.rs`: RAK3172-T/STM32WL interface, RF safety, PHY parameters, and
  short-lived radio session.
- `src/output.rs`: replaceable application output boundary, node identity, and
  sequence numbering.
- `src/diagnostics.rs`: non-blocking RTT startup and error reporting.
- `src/main.rs`: startup plus the sample -> emit -> scheduled-sleep loop.

## Verified hardware mapping

- module MCU: STM32WLE5CCU6
- BME688: I2C2, PA12/SCL, PA11/SDA, 100 kHz, address `0x76`
- I2C pull-ups: external 10 kohm resistors
- RF switch: PB8/RX and PC13/TX
- radio: EU868 high-power PA, DCDC, 3.0 V TCXO control

PB6/PB7 and I2C1 belong to an obsolete standalone-MCU revision and are not the
BME688 connection on this assembled RAK3172 board.

## Embassy revision

The project pins Embassy to commit
`3e99135a9e1bdfae06f5c9e88e9af96c8886d2eb`. The crates.io 0.6.0 source enables
STM32WL flash prefetch before lowering flash latency and reproducibly
hard-faults during startup on this board. The pinned upstream commit orders the
latency change safely.

See `THIRD_PARTY_NOTICES.md` for the attribution of the STM32WL radio interface
glue adapted from lora-rs.
