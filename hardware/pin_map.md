# VerveFlo — Pin Map

Board: **ESP32-S3 Dev Module**
Pins verified against the hardware sheet (firmware v3.1).

## Valves

Valve array index order in firmware: `VALVE_PINS[6] = { 26, 47, 39, 40, 41, 42 }`

| Index | Valve          | GPIO | Type            | Notes |
|-------|----------------|------|-----------------|-------|
| 0     | Fill Valve 1   | 26   | Normally-closed | Cuff 1 |
| 1     | Fill Valve 2   | 47   | Normally-closed | Cuff 2 |
| 2     | Fill Valve 3   | 39   | Normally-closed | Cuff 3 |
| 3     | Fill Valve 4   | 40   | Normally-closed | Cuff 4 |
| 4     | Fill Valve 5   | 41   | Normally-closed | Cuff 5 |
| 5     | Release Valve  | 42   | **Normally-open** | Logic inverted — energize (HIGH) to hold pressure, de-energize (LOW) to vent |

## Pump

| Component  | GPIO | Notes |
|------------|------|-------|
| Pump motor | 21   | Soft start via `analogWrite()` |

## Pressure sensor (HX710B)

| Signal     | GPIO | Notes |
|------------|------|-------|
| Dout       | 34   | Data out from sensor |
| SCK / CLK  | 33   | Clock |

Calibration (v3.1):
- `CALIB_M = +0.00000625` (raw increases with pressure — positive)
- `CALIB_B = 0.0`
- `CALIB_ZERO = 8388000` (atmosphere reference, re-zero with `zero` serial command)
- Overflow filter: discard raw > 16000000 (HX710B 2^24 saturation on fast release)

## Display

| Component | Interface | Notes |
|-----------|-----------|-------|
| SH1106 OLED 128x64 | I2C (HW) | U8g2 library |

## Buttons (active LOW)

| Button       | GPIO | Function |
|--------------|------|----------|
| BTN Start    | 35   | Play / start session |
| BTN Mode     | 36   | Cycle mode |
| BTN Pressure | 37   | Cycle pressure level |
| BTN Time     | 48   | Cycle session time |

## Misc

| Component   | GPIO | Notes |
|-------------|------|-------|
| LED         | 3    | Status LED |
| Battery ADC | 15   | Voltage divider — battery % |

## Power

- 2S 18650 Li-Ion battery
- 5V rail for pump + valves
