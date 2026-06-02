# VerveFlo — Setup & Flashing Guide

This guide explains everything needed to flash and run the main VerveFlo
firmware (`verveflo.ino`) on the device. No prior firmware experience needed —
just follow the steps in order.

---

## 1. What you need

**Hardware**
- The assembled VerveFlo board (ESP32-S3)
- A USB-C cable (data cable, not charge-only)
- A Windows / Mac / Linux computer
- A phone or laptop with WiFi (for first-time setup)

**Software**
- Arduino IDE 2.x — download free from https://www.arduino.cc/en/software

---

## 2. One-time computer setup

### 2.1 Add the ESP32 board support
1. Open Arduino IDE
2. Go to **File → Preferences**
3. In **Additional Boards Manager URLs**, paste:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
4. Click OK
5. Go to **Tools → Board → Boards Manager**, search **esp32**, install
   **"esp32 by Espressif Systems"**

### 2.2 Install the required libraries
Go to **Tools → Manage Libraries** and install:
| Library | Search for | By |
|---------|-----------|-----|
| U8g2 | `U8g2` | olikraus |
| Q2HX711 | `Queuetue HX711` | bogde / Queuetue |  https://github.com/queuetue/Q2-HX711-Arduino-Library

(WiFi, WebServer, DNSServer, Preferences come built in with the ESP32 core — nothing to install.)

---

## 3. Board settings (important)

Open `verveflo.ino`, then under **Tools** set exactly:

| Setting | Value |
|---------|-------|
| Board | **ESP32S3 Dev Module** |
| USB CDC On Boot | **Enabled**  ← critical, else serial won't work |
| Partition Scheme | Default 4MB with spiffs |
| Upload Speed | 921600 |

Then under **Tools → Port**, select the port that appears when you plug in
the board (e.g. `COM5` on Windows, `/dev/ttyACM0` on Linux).

---

## 4. Flash the firmware

1. Open `verveflo/verveflo.ino` in Arduino IDE
2. Confirm board settings (Section 3)
3. Click the **→ Upload** button (arrow, top-left)
4. Wait for **"Done uploading"** at the bottom

> If upload fails: hold the **BOOT** button on the board, press **RESET** once,
> release BOOT, then click Upload again.

---

## 5. First-time WiFi setup

The device hosts its own setup page on first power-up.

1. Power on the device
2. On your phone, open WiFi settings — connect to the hotspot named
   **`VerveFlo-Setup`**
3. A setup page should open automatically. If not, open a browser and go to:
   ```
   192.168.4.1
   ```
4. Pick your home/clinic WiFi from the list, enter the password, tap **Save**
5. The device restarts and connects to your WiFi
6. Its new IP address is shown on the **OLED screen** and in the Serial Monitor

To reset WiFi later: serial command `wificlear`, or hold the reset
sequence — device returns to `VerveFlo-Setup` hotspot mode.

---

## 6. Open the control app

On any phone or laptop **on the same WiFi**, open a browser and go to:
```
http://[device-IP]
```
(the IP shown on the OLED, e.g. `http://192.168.1.42`)

The web app lets you start/stop sessions, set pressure level, set time,
and choose mode. No app install — it runs in the browser.

---

## 7. Verify hardware (recommended once)

Open **Serial Monitor** (magnifier icon, top-right). Set:
- Baud: **115200**
- Line ending: **Newline**

Then type:
| Command | What it does |
|---------|--------------|
| `help` | Full command list |
| `zero` | Set pressure zero — do this with cuffs open to air |
| `selftest` | Auto-checks LED, battery, pressure, pump, all valves |
| `status` | Current device state |

If `selftest` reports all components OK and the web app loads, the device
is ready to use.

---

## 8. Package contents

| File / folder | What it is |
|---------------|------------|
| `verveflo/verveflo.ino` | Main firmware — the one you flash |
| `tools/` | Diagnostic sketches (only needed for troubleshooting) |
| `hardware/pin_map.md` | Pin assignments reference |
| `docs/` | Setup, calibration, troubleshooting docs |
| `README.md` | Overview |

---

## 9. Quick troubleshooting

| Symptom | Check |
|---------|-------|
| Nothing on Serial Monitor | USB CDC On Boot = Enabled, baud = 115200 |
| Upload fails | Hold BOOT + tap RESET, retry |
| Web page won't load | Device + phone on same WiFi? Use IP from OLED |
| Pressure reads wrong | Run `zero` with cuffs open to atmosphere |
| Pump runs, no inflation | Check cuff tubing + release valve seated |
| Can't find `VerveFlo-Setup` | Send `wificlear` over serial to reset WiFi |

For deeper hardware faults, flash the sketches in `tools/` — see their
header comments for command lists.
