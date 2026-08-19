# Firmware protocol v2

`telemetry-v1` remains the default and resolves the exact published
[`bme68x = 0.1.0`](https://crates.io/crates/bme68x/0.1.0) driver. Profile-v2 pins
the reviewed driver at immutable Git revision
[`64ee71d375958927aee2ae3d3405a090bb16ea08`](https://github.com/carlosgj94/bosch-bme68x-rust/commit/64ee71d375958927aee2ae3d3405a090bb16ea08)
and pins `vesta-protocol` from `vesta-receiver` at
[`82d12623ec2e58dc0048c0174018afb20ae7686c`](https://github.com/carlosgj94/vesta-receiver/commit/82d12623ec2e58dc0048c0174018afb20ae7686c).
These are the exact `Cargo.toml`/`Cargo.lock` dependencies, not local path
substitutions. The firmware package is version `0.2.0`, which profile-v2
`DeviceConfig` reports; v1 has no firmware-version field on its wire format.
The authoritative protocol-v2 byte table and golden frames are in the pinned
receiver's [`docs/PROTOCOL_V2.md`](https://github.com/carlosgj94/vesta-receiver/blob/82d12623ec2e58dc0048c0174018afb20ae7686c/docs/PROTOCOL_V2.md).

## Build modes

```bash
# Existing deployed protocol v1, one-minute forced-mode behavior
cargo build --release --locked

# Production profile v2 over LoRa; defaults to a 300-second start cadence
cargo build --release --locked --no-default-features --features profile-v2

# Laboratory profile v2 over USART2/PA2 only; defaults to 15 seconds
cargo build --release --locked --no-default-features --features profile-v2-uart,debug-sleep
```

The repository's target runner invokes `probe-rs`, so the corresponding flash
commands are:

```bash
# Deployed-compatible v1
cargo run --release --locked

# Profile-v2 LoRa, only after the matching receiver is deployed
VESTA_SCAN_INTERVAL_SECS=300 cargo run --release --locked --no-default-features --features profile-v2

# Laboratory UART capture; debug-sleep is deliberately not a power test
VESTA_SCAN_INTERVAL_SECS=15 cargo run --release --locked --no-default-features --features profile-v2-uart,debug-sleep
```

None of these `cargo run` commands has been executed during this verification.

`VESTA_SCAN_INTERVAL_SECS` changes the profile cadence at compile time. UART
builds accept 15 seconds or longer. LoRa builds reject values below 180 seconds
to keep the four-fragment profile plus repeated records inside the EU868 1%
airtime budget; the default is 300 seconds. For LoRa, `DeviceConfig` and
`DeviceHealth` are emitted on scan zero and every six scans. UART training
repeats `DeviceConfig` every scan and `DeviceHealth` every six scans. Every
scan emits all four deterministic profile-fragment windows.

At SF7/BW125/CR4/5 with an explicit header and PHY CRC, a complete profile
uses 231, 231, 231, and 137-byte packets. Their airtimes are 363.776, 363.776,
363.776, and 225.536 ms: 830 payload bytes and 1.316864 s per scan. A 231-byte
config costs 363.776 ms and the 102-byte base health record costs 174.336 ms.
Repeating both every six scans gives 1.406549 s/scan long-term average airtime:
about 0.7814% at the enforced 180-second minimum and 0.4689% at the 300-second
default, before retries. These are engineering airtime-budget calculations,
not certification. Spain's [CNAF UN-39 primary rule](https://www.boe.es/eli/es/o/2021/12/16/etd1449)
covers 868.000--868.600 MHz at up to 25 mW ERP and requires an access/mitigation
technique or the alternative duty-cycle limit of at most 1%. Product-specific
regulatory applicability and conformity still require specialist review.

Exactly one of `telemetry-v1` and `profile-v2` must be active. UART training
does not compile or own a LoRa output path and advertises only the UART route in
`DeviceConfig`.

## HP-354 acquisition

The initial neutral exploratory profile is Bosch's documented HP-354 standard
profile, not a fire classifier. The exact ordered values come from the
[Bosch BME AI-Studio Manual, BST-BME688-AN001-00 v1.6.0, page 70](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bme688-an001.pdf):

Bosch's [board-configuration guidance](https://www.bosch-sensortec.com/software/bme/docs/process-steps/configure-board.html)
calls HP-354 its standard default, a versatile short profile suitable as a
starting point. That is the selection rationale; it is not evidence of a
wildfire-specific profile or classifier.

- environmental config: humidity x1, pressure x16, temperature x2, filter off;
- target temperatures: `320,100,100,100,200,200,200,320,320,320` degrees C;
- TPHG repetition multipliers: `5,2,10,30,5,5,5,5,5,5` (sum 77);
- requested shared wait: 99 ms;
- read-back shared-wait register: expected `0x73`, exactly 97,308 us;
- TPHG duration: 41,590 us;
- exact represented profile duration: 10,695,146 us.

The firmware polls every 100 ms with both a 150-poll and 15-second bound. The
fixed-capacity driver collector reassembles all ten gas indexes across the
three hardware field slots and records duplicates, overwrites, discontinuities,
rollovers, invalid indexes, dummy/intermediate fields, gas validity, heater
stability, and observation overflow. A bus/read/timeout failure returns a
partial scan; it does not discard already collected steps. Every scan exit
attempts `OperationMode::Sleep` because Parallel mode otherwise repeats.

After verified Sleep and immediately before anchoring/triggering each scan, one
bounded three-slot field read drains prior-cycle `NEW_DATA`. A drain error
aborts the acquisition attempt. A nonzero drain count sets
`STALE_PRE_SCAN_FIELDS` (collection flag bit 13); the exact count is
saturating-added to `intermediate_field_count`, whose semantics are therefore
the total discarded nonterminal fields (collector dummy/intermediate fields
plus stale pre-scan fields). Saturation also sets `OBSERVATION_OVERFLOW`. Normal
drains are expected to return zero because field reads consume `NEW_DATA`.

Before every heater scan, the firmware reads back the complete environmental
and heater configuration. A mismatch triggers one bounded reprogram/readback
attempt before Parallel mode. A successful restore is exact and may be used,
but the scan carries `SENSOR_RECONFIGURED` so the server can break or reset its
history. An unsuccessful restore does not trigger the heater: it produces an
explicit empty/partial failed scan with `CONFIG_MISMATCH` and `config_id = 0`.
Trigger/read/sleep errors also cause immediate readback/recovery after the
partial scan. The current record retains a copy of the exact pre-trigger
metadata and configuration ID; post-scan recovery only affects later scans and
can never relabel measurements already taken. `SENSOR_RECONFIGURED` therefore
means restoration before the trigger. Post-scan recovery status and errors are
reported in diagnostics and health accounting. A successful post-scan restore
is retained and marks the next exactly verified scan with
`SENSOR_RECONFIGURED`, so the receiver resets temporal history at the first
usable post-recovery profile. The marker remains pending through drain/trigger
failure, failed or partially applied configuration programming, locally
unusable scans, codec failure, and partial output. It is
cleared only after every fragment of a locally usable marked profile has been
confirmed `TX_DONE`/UART-written (optional later health failure does not revoke
local completion). This is not proof of Raspberry Pi reception. The matching
receiver additionally resets/refuses temporal history when usable v2 scan
sequences are not wrapping-`+1` contiguous, covering a whole marked profile
lost over RF. Configuration preflight/recovery time is excluded from scan-start
uptime, per-step offsets, and scan duration; a preflight failure retains its
scheduled-attempt timestamp because no heater start exists.

If the normal post-Parallel Sleep request and the bounded post-scan recovery
still cannot prove Sleep, firmware makes exactly three additional stop-only
commands spaced by 10 ms. Every failed I2C attempt is saturating-counted and
the original scan/Sleep fault remains analytically fatal; no extra LoRa record
is emitted inside this local retry. RTT reports the attempt count and whether a
retry finally confirmed Sleep. After all three failures the firmware emits its
normal failed-scan/health batch rather than looping indefinitely. Safe BME688
heater shutdown is then unprovable because this PCB has no sensor power switch
or dedicated sensor-reset gate; that residual failure requires hardware reset.

Each step `offset_us` is the bounded MCU poll/read observation time measured
from the trigger anchor, not an exact gas-conversion timestamp. The exact
configured heater timing remains in `DeviceConfig` through the repetition
multiplier, shared wait, TPHG, and effective step duration.

All set bits in `heater_readback_valid_bitmap` mean that the raw
IDAC/RES_HEAT/GAS_WAIT bytes were successfully read into that step descriptor.
Exact configuration validation compares RES_HEAT, GAS_WAIT, shared/control,
operation-mode, and environmental registers. IDAC is preserved as raw metadata,
not compared with a programmed expectation, because this driver does not
program IDAC.

Before encoding each verified scan, the in-memory `DeviceConfig` is rebuilt
from that scan's immutable pre-trigger readback snapshot. If any raw metadata
changes, the firmware recomputes
`config_id` and forces the new `DeviceConfig` into that scan's output batch
instead of waiting for the normal six-scan repetition boundary. Initial,
periodic, and changed definitions remain delivery-pending through codec errors,
configuration-mismatch scans, and output failure; the pending state clears
only after the config-first record is confirmed complete, so a failed cadence
packet is retried on the next verified scan rather than delayed six scans. The
`CONFIG_REPEAT` marker is tracked per exact `config_id`: startup and the first
successfully completed definition after any mid-boot ID change are marked new,
while only later records of that already delivered ID are marked repeats.

## Identity, health, and power ownership

RCC reset status is captured before reset flags are cleared. STM32 hardware RNG
generates an eight-byte boot nonce once; its driver is then dropped and clock
disabled before scheduled sleep. RNG failure is represented as an unavailable
boot ID rather than fabricated uniqueness. Health counters saturate and include
sensor scans, incomplete profiles, I2C errors, LoRa errors, dropped profiles
and fragments, and overwritten fields.

A `successful_sensor_scan` uses the same fail-closed quality boundary as the
receiver: exact configuration, no sensor/I2C error, complete structure and
finish, every gas-valid and heater-stable bit, and no overwrite, invalid index,
measurement discontinuity, rollover, counter overflow, or stale pre-scan
field. Expected duplicate observations and discarded Parallel dummy fields are
nonfatal. Every other logical attempt increments both `failed_sensor_scans` and
`incomplete_profiles`.

The firmware build ID is the first 64 bits of the exact Git commit. The build
script includes untracked files in its dirty check and asks Cargo to rerun for
recursive source/build inputs, worktree HEAD/index, packed refs, and the
resolved symbolic branch ref, so a commit advance cannot silently reuse an old
build ID.

`failed_sensor_scans` and `incomplete_profiles` use that same fail-closed
predicate. A timeout with all ten slots, unstable heater, recovered preflight
I2C error, stale drain, overwrite/discontinuity, or structurally complete scan
with invalid gas is still incomplete; none is hidden by slot count alone.

If BME address probing, driver initialization, HP-354 setup/readback, retained
metadata, or telemetry setup fails before the first scan, the node emits only a
102-byte `DeviceHealth` record—never a fabricated `DeviceConfig` or profile.
The common header retains the exact node ID, hardware-RNG boot-ID status, reset
cause, uptime, and a wrapping health-retry sequence. `config_id = 0`,
`profile_id = 0`, and `profile_version = 0` mean “no verified sensor profile is
available”; the pinned codec and receiver accept and store this sentinel.
Probe failure counts the two concrete failed address transactions and RTT logs
both address errors plus SCL/SDA levels. Other BME failures retain the exact
operation code and count I2C errors when the driver identifies one.
Sensor-operation codes are `1=initialization`, `2=environmental configuration`,
`3=heater configuration`, `4=configuration readback`, `5=Parallel trigger`,
`6=data read`, `7=return to Sleep`, and `8=pre-trigger field drain`; startup
sentinels use the separate `0x0101`/`0x0201`/`0x0202` ranges.

Recoverable startup states are retried once per configured scan cadence after
their health record is attempted. Probe failure returns `SensorBus`, while
setup/readback/metadata/telemetry failures retain `Sensor`; retries therefore
reuse the original Embassy ownership tokens and keep only short-lived concrete
I2C/radio/UART drivers. Reporter sequence, cumulative I2C count, and output
failure counters survive these retries. The one reset-required subset is an
error inside `Bme68x::new`: the pinned driver consumes its register interface
and does not return it on construction failure, so the firmware cannot soundly
recreate the peripheral tokens. That state continues periodic degraded health
transmission until reset rather than using unsafe ownership reconstruction.

Health-only retries use the configured cadence (300 seconds by default and at
least 180 seconds for LoRa; 15 seconds by default for UART). Each attempt
constructs and drops the concrete radio/UART driver before the timer wait, so
STOP2 eligibility is retained. Every output failure is counted locally, logged,
and retried. LoRa failures are additionally exposed in cumulative
`radio_tx_errors`/`last_radio_error` on the next successfully delivered health
record. UART failures are not mislabeled as radio failures; their count remains
an RTT transport diagnostic because this protocol revision has no generic UART
error counter.

I2C2, SUBGHZSPI, and USART2 concrete peripheral drivers exist only during
transactions or output. PB8/PC13 remain persistent push-pull GPIO outputs at
low/low throughout idle, avoiding an undefined high-impedance RF-switch state;
the UART-only training build also owns and holds both pins low for its entire
runtime. Retaining those GPIO outputs adds no RCC STOP constraint. Production
remains structurally STOP2-eligible after each batch;
`debug-sleep` is intentionally not evidence of STOP2 or current. MCU temperature
and regulated VDD are omitted because factory-calibrated ADC conversion is not
implemented. The PCB cannot measure raw battery voltage or state of charge.
The RAK3172-T PB0/VDDTCXO supply is driven by the STM32WL radio DIO3 command at
3.0 V, not held as an ordinary GPIO; cold radio sleep plus final RFRST provides
its teardown. All STOP2 statements here are structural and still require a
hardware power trace.

## UART training framing

Each unchanged protocol-v2 record gets a big-endian CRC-32/ISO-HDLC trailer
(polynomial `0x04C11DB7`, reflected `0xEDB88320`, init/xorout `0xffffffff`), is
COBS encoded, and ends with `0x00`. USART2/PA2 runs at 115200 baud, 8 data bits,
no parity, and one stop bit (8N1). Host fixtures cover the standard
`123456789 -> 0xcbf43926` CRC, maximum frame, round trip, and single-byte
corruption rejection.

## Hardware boundary

Protocol-v1 has already been flashed and received end to end with valid LoRa
PHY CRC and successful Rust decoding. The target builds and host tests here do
not prove a physical profile-v2 ten-step capture, USART bytes, v2 LoRa TX
completion/reception, STOP2 entry, or current. Do not flash profile-v2 until the
matching receiver decoder is deployed and its golden fixtures agree. Those v2
hardware checks remain required after integration.
