# ESPHome MS5837

Native ESPHome component for the **MS5837-02BA** and **MS5837-30BA** pressure / temperature sensor.  
Perfect for measuring water depth, tank/reservoir level, or altitude.

> **Note:** The old `platform: custom` + `MS5837_Component.h` approach is no longer used.  
> This is a proper ESPHome external component — no manual includes or Arduino libraries required.

---

## Wiring

<img src="/images/MS5837_Pinout.png" width="400px">
<img src="/images/MS5837_Circuit.png" width="400px">
<img src="/images/SampleBoard.png" width="400px">

Pull-up resistors and decoupling capacitor values depend on your installation.  
A 4.7 kΩ pull-up and 1 µF decoupling cap work well even over a 15-ft cable.

---

## Quick-start YAML

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/stm91/ESPHome_MS5837
      ref: main
    components: [ms5837]

i2c:
  sda: 20
  scl: 21
  scan: true

sensor:
  - platform: ms5837
    variant: 02ba          # 02ba or 30ba — set explicitly to avoid PROM auto-detect issues
    mode: 2                # 0 = raw, 1 = altitude, 2 = depth/level
    osr: 0                 # ADC oversampling (0 = 8192, highest accuracy)
    avg_count: 3           # average 3 readings per update cycle
    update_interval: 10s

    atmospheric_pressure: 101325.0   # Pa — your local air pressure for zero calibration

    temperature:
      name: "Water Temperature"
    pressure:
      name: "Raw Pressure"
    altitude:
      name: "Water Level"    # reports depth/level in metres when mode: 2
```

A fully-annotated example is in [`example/example.yaml`](example/example.yaml).

---

## Configuration reference

### Required I2C block

```yaml
i2c:
  sda: <pin>
  scl: <pin>
  scan: true
```

Default I2C address: **0x76**.  Override with `address: 0x77` inside the sensor block if needed.

---

### `platform: ms5837` options

| Key | Default | Description |
|-----|---------|-------------|
| `variant` | `auto` | Sensor variant: `auto` (PROM detect), `02ba`, or `30ba`. Set explicitly when `auto` is unreliable. |
| `mode` | `0` | `0` = raw (temp + pressure only) · `1` = altitude · `2` = depth / water level |
| `osr` | `0` | ADC oversampling index — see table below |
| `avg_count` | `1` | Number of ADC readings averaged per update (higher = smoother, slower) |
| `update_interval` | `60s` | How often to poll the sensor |
| `atmospheric_pressure` | `101325.0` | Local air pressure in **Pa** — used as the depth=0 baseline in `mode: 2` |
| `temp_offset` | `0.0` | Temperature trim in **°C** |
| `press_offset` | `0.0` | Pressure trim in **hPa** applied after atmospheric correction |
| `fluid_density` | `997.0` | Fluid density in **kg/m³** (`997` = freshwater, `1025` = seawater) |
| `temperature` | — | Optional temperature sensor sub-block |
| `pressure` | — | Optional pressure sensor sub-block (hPa) |
| `altitude` | — | Optional altitude/depth/level sensor sub-block (m) |

#### OSR index table

| `osr` | Oversampling | Conversion time |
|-------|-------------|-----------------|
| `0`   | 8192        | ~18 ms (default, most accurate) |
| `1`   | 4096        | ~9 ms  |
| `2`   | 2048        | ~5 ms  |
| `3`   | 1024        | ~3 ms  |
| `4`   | 512         | ~2 ms  |
| `5`   | 256         | ~1 ms  |

---

## Modes

### `mode: 0` — Raw
Reports temperature (°C) and pressure (hPa) only.  The `altitude` sub-sensor is ignored.

### `mode: 1` — Altitude
Calculates altitude using the barometric formula.  
Set `atmospheric_pressure` to your current sea-level pressure for true altitude; leave at `101325` for pressure altitude.

### `mode: 2` — Depth / Water level *(reservoir use-case)*
Calculates depth/level from the pressure difference between the sensor and `atmospheric_pressure`.  
The `altitude` sensor entity reports this value in metres.

---

## Calibrating zero for reservoir level

1. **Set `atmospheric_pressure`** to your actual local air pressure in Pa (e.g. from a nearby weather station).  
   This is the primary zero-point adjustment — get it right and `press_offset` won't be needed.

2. **Fine-tune with `press_offset`** if a small residual offset remains after installation.  
   With the reservoir at your chosen "zero" reference level (e.g. empty), read the reported level,  
   then set `press_offset` to `-(reading_m × fluid_density × 9.81 / 100)` (result in hPa).  
   Example: sensor reads 0.05 m when empty → `press_offset: -4.89`

3. **Only publish changes** — add `filters: - delta: 0.005` to the altitude sensor sub-block to  
   suppress updates when the value hasn't changed meaningfully.

---

## Updating atmospheric pressure at runtime

Use an ESPHome [`number`](https://esphome.io/components/number/) entity backed by a `template` platform
and call `set_atmospheric_pressure()` through a lambda:

```yaml
number:
  - platform: template
    name: "Atmospheric Pressure Reference"
    id: atm_pressure_ref
    unit_of_measurement: "Pa"
    mode: box
    min_value: 90000
    max_value: 110000
    step: 1
    initial_value: 101325
    optimistic: true
    restore_value: true
    set_action:
      - lambda: |-
          id(ms5837_sensor).set_atmospheric_pressure(x);

sensor:
  - platform: ms5837
    id: ms5837_sensor
    # ... rest of config
```

---

## Hardware variants

| Variant | Range | Typical use |
|---------|-------|-------------|
| MS5837-**02BA** | 0–2 bar (0–10 m fresh water) | Reservoir level, shallow depth |
| MS5837-**30BA** | 0–30 bar (0–300 m) | Deep water, Blue Robotics Bar30 |

Set `variant: 02ba` or `variant: 30ba` explicitly to bypass PROM auto-detection,  
which can be unreliable on some production batches.

---

## Second-order temperature compensation

The component implements the full MS5837 §8.2 two-stage compensation:

- **First order** — applied at all temperatures.
- **Second order (low-T)** — additional correction terms activate for T < 20 °C to reduce non-linearity.
- **Second order (very-low-T)** — further correction terms activate for T < −15 °C.

This is handled entirely in firmware; no user configuration is required.

---

## References

- [MS5837-02BA Datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5837-02BA01&DocType=DS&DocLang=English)
- [MS5837-30BA Datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5837-30BA&DocType=DS&DocLang=English)
- [ESPHome External Components](https://esphome.io/components/external_components.html)

## Thanks

Originally inspired by Blue Robotics' [MS5837 Arduino library](https://github.com/bluerobotics/BlueRobotics_MS5837_Library).
