# AGENTS.md

## What this repo is
- Custom ESPHome component implementing an `ms5837` sensor platform (`esphome/components/ms5837`).
- Runtime logic is C++ (`ms5837.cpp`/`ms5837.h`); YAML schema/codegen glue is Python (`sensor.py`).
- `manifest.json` declares domain `ms5837` and dependency on `i2c`.

## Big-picture architecture
- Config entrypoint: `sensor.py:CONFIG_SCHEMA` defines options (`update_interval`, `mode`, `osr`, `avg_count`, and optional `temperature`/`pressure`/`altitude` sensors).
- Codegen handoff: `sensor.py:to_code()` constructs `MS5837Sensor(update_ms, mode, osr)`, calls `i2c.register_i2c_device()` (wires the bus/address), then registers child sensors.
- Runtime class: `MS5837Sensor` inherits `PollingComponent` + `i2c::I2CDevice` (`ms5837.h`).
- Poll cycle: `update()` -> `read_and_calc_values()` (avg_runs_ loop) -> `calculate()` -> unit conversion -> `publish_state()` (`ms5837.cpp`).
- Failure model is explicit: hard init failures call `mark_failed()`; transient read failures publish `NAN` and set warning status.

## Critical implementation patterns
- `setup()` performs a bounded I2C probe loop (300 ms max) to avoid boot lockups when bus lines are bad.
- ADC conversion waits use `wd_delay_ms()` slices and `App.feed_wdt()` to stay watchdog-friendly during blocking delays.
- OSR is an index (`0..5`), not a raw oversampling value; conversion timing comes from `CONVERSION_TIME[]`.
- Altitude/depth share `altitude_sensor`; `mode` controls whether the third channel is altitude, depth, or unused.
- Internal base units: temperature in °C and pressure in hPa; offsets are applied before output unit conversion.
- `calculate()` applies **both** first-order and second-order compensation (MS5837-30BA §8.2) before computing pressure; the second-order terms activate for T < 20 °C and extend at T < -15 °C.
- `read_and_calc_values()` loops `avg_runs_` times and averages raw D1/D2 ADC values before compensation, matching the legacy `SetResultsAvgCount()` behaviour.

## Conventions specific to this codebase
- Namespace must match folder/domain: `ms5837` (commented in `sensor.py`).
- `MS5837Sensor` Python class declaration must list **both** `cg.PollingComponent` and `i2c.I2CDevice` as parents so `register_i2c_device()` can call `set_i2c_bus`/`set_i2c_address`.
- Even optional outputs are pre-allocated in C++ (`new sensor::Sensor()` members in `ms5837.h`), then conditionally registered in Python.
- Status signaling style: use ESPHome status APIs (`status_set_warning`, `status_clear_warning`) instead of silent failures.
- Logging tags follow `"ms5837.sensor"` and emphasise recoverability (retry/setup paths).
- Schema helpers: use `cv.polling_component_schema("60s")` and `i2c.i2c_device_schema(0x76)` — do not add `update_interval` manually or extend `cv.COMPONENT_SCHEMA`.

## Integration points and boundaries
- External boundary is ESPHome sensor platform YAML (`platform: ms5837`) loaded via `external_components` — see `example/example.yaml` for the correct modern usage.
- Hardware boundary is I2C at address `0x76` (default) with PROM read + CRC check before normal operation.
- Unit/mode constants are C++ macros in `ms5837.h`; Python schema accepts validated integers for `mode` (0–2) and `osr` (0–5).

## Workflow notes for agents
- Source-of-truth for behaviour is `esphome/components/ms5837/*`.
- `README.md` describes an older `platform: custom` + `MS5837_Component.h` flow that no longer applies; ignore it for implementation guidance.
- When changing config UX, update both `sensor.py` schema and C++ constructor/setter paths together.
- Validate edits by running ESPHome config/build in a host project that imports this component as an external component (see `example/example.yaml`).

