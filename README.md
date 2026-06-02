# VerveFlo

Pneumatic Compression Therapy Device firmware
ESP32-S3 · Arduino IDE

## Hardware
- ESP32-S3 Dev Module
- SH1106 OLED 128x64 (I2C)
- HX710B Pressure Sensor
- 5x Fill Solenoid Valves + 1x Release Valve (normally-open)
- 5V DC Pump Motor
- 2S 18650 Li-Ion Battery

## Pin Map
| Component       | GPIO |
|----------------|------|
| Fill Valve 1   | 26   |
| Fill Valve 2   | 47   |
| Fill Valve 3   | 39   |
| Fill Valve 4   | 40   |
| Fill Valve 5   | 41   |
| Release Valve  | 42   |
| Pump Motor     | 21   |
| HX710B Dout    | 34   |
| HX710B SCK     | 33   |
| LED            | 3    |
| Battery ADC    | 15   |
| BTN Start      | 35   |
| BTN Mode       | 36   |
| BTN Pressure   | 37   |
| BTN Time       | 48   |

## Libraries Required
- U8g2 by olikraus
- Q2HX711 by bogde (Queuetue_HX711_Library)
- WiFi / WebServer / DNSServer / Preferences — built into ESP32 core

## Board Settings
- Board: ESP32S3 Dev Module
- USB CDC On Boot: Enabled
- Partition: Default 4MB with spiffs

## Files
| File | Purpose |
|------|---------|
| verveflo.ino | Main firmware — flash this |
| verveflo_hwtest.ino | Hardware test — no libraries needed |
| verveflo_pressure_cal.ino | Pressure sensor calibration |
| verveflo_valve_test.ino | Valve test with OLED |

## First Time Setup
1. Flash verveflo.ino
2. Connect phone to WiFi hotspot: VerveFlo-Setup
3. Open 192.168.4.1 → enter your WiFi password
4. Open browser → http://[device IP]
5. Serial: type zero at atmosphere
6. Serial: type selftest to verify hardware

## Serial Monitor
115200 baud · Newline line ending
Type help for full command list
