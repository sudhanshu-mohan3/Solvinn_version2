/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║              VerveFlo  v3.1  —  Complete Firmware        ║
 * ║         ESP32-S3 · Arduino IDE · All-in-One              ║
 * ╠══════════════════════════════════════════════════════════╣
 * ║  FIXES v3.1:                                             ║
 * ║  • Release valve (VR) logic inverted (normally-open)     ║
 * ║  • CALIB_M positive (raw increases with pressure)        ║
 * ║  • CALIB_ZERO offset in readPressure()                   ║
 * ║  • HX710B overflow filter (raw > 16000000)               ║
 * ║  • Motor soft start via analogWrite (no ledcWrite)       ║
 * ║  • 'zero' serial command for live calibration            ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * LIBRARIES (Tools → Manage Libraries):
 *   U8g2              by olikraus
 *   Q2HX711           by bogde (search Queuetue)
 *   WiFi/WebServer/DNSServer/Preferences → built into ESP32 core
 *
 * BOARD SETTINGS:
 *   Board             : ESP32S3 Dev Module
 *   USB CDC On Boot   : Enabled  ← MUST or serial won't work
 *   Partition Scheme  : Default 4MB with spiffs
 *   Upload Speed      : 921600
 *
 * PIN MAP (verified from hardware sheet):
 *   Fill Valve 1  GPIO 26    Release Valve  GPIO 42 (normally-open)
 *   Fill Valve 2  GPIO 47    Pump Motor     GPIO 21
 *   Fill Valve 3  GPIO 39    LED            GPIO 3
 *   Fill Valve 4  GPIO 40    HX710B Dout    GPIO 34
 *   Fill Valve 5  GPIO 41    HX710B SCK     GPIO 33
 *   BTN Start     GPIO 35    Battery ADC    GPIO 15
 *   BTN Mode      GPIO 36
 *   BTN Pressure  GPIO 37
 *   BTN Time      GPIO 48
 *
 * SERIAL COMMANDS (115200 baud, Newline):
 *   help / status / start / pause / stop / release
 *   pressure / battery / valves / buttons / pins / ip
 *   zero      → set pressure zero at atmosphere
 *   v1-v5 on/off/open/close/0-255
 *   vr on/off/open/close
 *   pump on/off / motor 0-255
 *   pstream / ptest / vsweep / selftest / alloff
 *   led on/off/blink/auto
 *   mode 1-4 / pset 0-2 / time 0-2 / hold N / gap N
 *   wificlear / reboot
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Q2HX711.h>

// ═══════════════════════════════════════════════════════════
//  *** CHANGE THESE TO YOUR WIFI CREDENTIALS ***
//  OR leave blank and use captive portal on first boot
// ═══════════════════════════════════════════════════════════
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

// ═══════════════════════════════════════════════════════════
//  PINS
// ═══════════════════════════════════════════════════════════
const uint8_t VALVE_PINS[6] = { 26, 47, 39, 40, 41, 42 };
//                               V1   V2   V3   V4   V5   VR
// NOTE: VR (index 5, GPIO 42) is NORMALLY-OPEN — logic inverted
#define RELEASE_VALVE_IDX  5
#define NUM_INFLATE_VALVES 5
#define PUMP_PIN    21
#define LED_PIN     3
#define HX_DATA     34
#define HX_CLK      33
#define BATT_PIN    15
#define BTN_START   35
#define BTN_MODE    36
#define BTN_PRESS   37
#define BTN_TIME    48

// Battery divider (100k + 100k = ratio 2.0)
#define BATT_DIVIDER  2.0f
#define BATT_MAX_V    4.2f
#define BATT_MIN_V    3.0f

// ═══════════════════════════════════════════════════════════
//  OBJECTS
// ═══════════════════════════════════════════════════════════
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
Q2HX711 hx711(HX_DATA, HX_CLK);
WebServer server(80);
DNSServer dnsServer;
Preferences prefs;

// ═══════════════════════════════════════════════════════════
//  WIFI STATE
// ═══════════════════════════════════════════════════════════
bool   g_wifiConnected = false;
bool   g_apMode        = false;
String g_savedSSID     = "";
String g_savedPass     = "";
#define AP_SSID  "VerveFlo-Setup"
#define AP_PASS  ""
#define DNS_PORT 53

// ═══════════════════════════════════════════════════════════
//  PRESSURE CALIBRATION
//  IMPORTANT: CALIB_M is POSITIVE — raw increases with pressure
//  Run 'zero' serial command at atmosphere to set CALIB_ZERO
// ═══════════════════════════════════════════════════════════
float CALIB_M    =  0.00000625f;  // positive slope
float CALIB_B    =  0.0f;
long  CALIB_ZERO =  8388000L;     // update via 'zero' serial command
float g_pressure =  0.0f;

// ═══════════════════════════════════════════════════════════
//  SYSTEM STATE
// ═══════════════════════════════════════════════════════════
enum SystemState {
  STATE_BOOT,
  STATE_READY,
  STATE_RUNNING,
  STATE_PAUSED,
  STATE_RELEASING,
  STATE_DEFLATE_WAIT
};
SystemState g_sysState = STATE_BOOT;

// ═══════════════════════════════════════════════════════════
//  SESSION SETTINGS
// ═══════════════════════════════════════════════════════════
const float    PRESSURE_LEVELS[] = { 40.0f, 70.0f, 100.0f };
const char*    PRESSURE_LABELS[] = { "LOW",  "MED",  "HIGH" };
const uint16_t TIME_OPTIONS[]    = { 10, 20, 30 };
const char*    TIME_LABELS[]     = { "10m", "20m", "30m" };

const uint8_t MODE_SEQS[4][5] = {
  { 0,1,2,3,4 },   // Mode 1: Distal → Proximal
  { 4,3,2,1,0 },   // Mode 2: Proximal → Distal
  { 0,2,4,1,3 },   // Mode 3: Alternating
  { 0,4,1,3,2 }    // Mode 4: Wave
};
const char* MODE_NAMES[] = { "DISTAL", "PROXIM", "ALTERN", "WAVE" };

uint8_t  g_pressureIdx   = 1;       // default MED
uint8_t  g_timeIdx       = 0;       // default 10 min
uint8_t  g_mode          = 0;       // default Mode 1
uint32_t g_holdMs        = 8000UL;  // inflate hold per cuff
uint32_t g_deflateWaitMs = 2000UL;  // deflate gap between cuffs

// ═══════════════════════════════════════════════════════════
//  VALVE PWM RAMP (non-blocking)
//  Fill valves: 0=off, 255=open (normally-closed)
//  Release valve: 0=open, 255=closed (normally-open, INVERTED)
// ═══════════════════════════════════════════════════════════
#define RAMP_DURATION_MS   2500UL
#define RAMP_STEPS         256
#define RAMP_STEP_INTERVAL (RAMP_DURATION_MS / RAMP_STEPS)  // ~9ms

struct ValveRamp {
  bool          active      = false;
  bool          opening     = false;
  uint8_t       duty        = 0;
  uint8_t       valveIdx    = 0;
  unsigned long lastStepTime= 0;
};
ValveRamp g_ramp;

// ═══════════════════════════════════════════════════════════
//  MOTOR SOFT START (analogWrite, non-blocking)
//  5 counts per step × 30ms = ~1.5s full ramp
// ═══════════════════════════════════════════════════════════
uint8_t       g_motorDuty    = 0;
uint8_t       g_motorTarget  = 0;
bool          g_motorRamping = false;
unsigned long g_motorRampT   = 0;

// ═══════════════════════════════════════════════════════════
//  SESSION STATE
// ═══════════════════════════════════════════════════════════
unsigned long g_sessionStartTime  = 0;
unsigned long g_sessionDurationMs = 0;
uint8_t       g_cuffStep          = 0;
bool          g_cuffInflated      = false;
unsigned long g_inflateHoldStart  = 0;
unsigned long g_deflateWaitStart  = 0;
uint8_t       g_releaseStep       = 0;
bool          g_releaseWaiting    = false;
unsigned long g_releaseStepStart  = 0;
#define RELEASE_STEP_WAIT_MS 3000UL

// ═══════════════════════════════════════════════════════════
//  BUTTONS (millis debounce, no blocking)
// ═══════════════════════════════════════════════════════════
#define DEBOUNCE_MS  50UL
#define LONGPRESS_MS 2000UL

struct Button {
  uint8_t       pin;
  bool          lastRaw        = HIGH;
  bool          state          = HIGH;
  bool          longFired      = false;
  unsigned long pressTime      = 0;
  unsigned long lastChangeTime = 0;
};
Button g_btns[4] = {{BTN_START},{BTN_MODE},{BTN_PRESS},{BTN_TIME}};
bool g_evt_start    = false;
bool g_evt_pressure = false;
bool g_evt_time     = false;
bool g_evt_mode     = false;
bool g_evt_reset    = false;

// ═══════════════════════════════════════════════════════════
//  LED PATTERNS (non-blocking)
// ═══════════════════════════════════════════════════════════
enum LedPattern { LED_OFF, LED_SOLID, LED_SLOW, LED_FAST, LED_PULSE };
LedPattern    g_ledPat    = LED_OFF;
bool          g_ledForced = false;
unsigned long g_ledTime   = 0;
bool          g_ledState  = false;
uint8_t       g_ledStep   = 0;

void setLed(LedPattern p) { if (!g_ledForced) g_ledPat = p; }

void updateLED() {
  if (g_ledForced) return;
  unsigned long now = millis();
  switch (g_ledPat) {
    case LED_OFF:   digitalWrite(LED_PIN, LOW);  break;
    case LED_SOLID: digitalWrite(LED_PIN, HIGH); break;
    case LED_SLOW:
      if (now-g_ledTime >= 1000) {
        g_ledState = !g_ledState;
        digitalWrite(LED_PIN, g_ledState);
        g_ledTime = now;
      } break;
    case LED_FAST:
      if (now-g_ledTime >= 100) {
        g_ledState = !g_ledState;
        digitalWrite(LED_PIN, g_ledState);
        g_ledTime = now;
      } break;
    case LED_PULSE:
      if (now-g_ledTime >= 80) {
        g_ledTime = now;
        g_ledStep = (g_ledStep + 1) % 25;
        digitalWrite(LED_PIN, (g_ledStep==0 || g_ledStep==2));
      } break;
  }
}

// ═══════════════════════════════════════════════════════════
//  BATTERY (GPIO 15, 4-bar display)
// ═══════════════════════════════════════════════════════════
float   g_battV    = 4.2f;
uint8_t g_battPct  = 100;
uint8_t g_battBars = 4;
unsigned long g_lastBattTime = 0;
#define BATT_INTERVAL_MS 10000UL

void readBattery() {
  uint32_t sum = 0;
  for (uint8_t i=0; i<8; i++) { sum += analogRead(BATT_PIN); delay(2); }
  float raw = sum / 8.0f;
  float v   = (raw / 4095.0f) * 3.3f * BATT_DIVIDER;
  g_battV   = v;
  float pct = (v - BATT_MIN_V) / (BATT_MAX_V - BATT_MIN_V) * 100.0f;
  g_battPct  = (uint8_t)constrain(pct, 0, 100);
  g_battBars = (g_battPct>=80)?4:(g_battPct>=55)?3:(g_battPct>=30)?2:(g_battPct>=10)?1:0;
}

// ═══════════════════════════════════════════════════════════
//  DISPLAY TIMING
// ═══════════════════════════════════════════════════════════
unsigned long g_lastDisplay  = 0;
unsigned long g_lastPressRead= 0;
#define DISPLAY_MS  250UL
#define PRESS_MS    200UL

// Boot animation
uint8_t       g_bootStep     = 0;
unsigned long g_bootStepTime = 0;
#define BOOT_STEP_MS  80UL
#define BOOT_STEPS    28

// Cuff circle animation
uint8_t       g_cuffAnim[5]  = {0,0,0,0,0};
unsigned long g_lastAnim     = 0;
#define ANIM_MS 80UL

// ═══════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════
void startOpenValve(uint8_t idx);
void startCloseValve(uint8_t idx);
void allValvesOff();
void setPump(bool on);
void updateMotorRamp();
float readPressure();
void startSession(); void pauseSession();
void resumeSession(); void beginRelease(); void resetSystem();
void advanceCuffStep();
void updateRamp(); void updateSession(); void updateReleaseSequence();
void pollButtons(); void handleEvents();
void updateDisplay();
void drawBootScreen(); void drawReadyScreen();
void drawRunScreen(); void drawPausedScreen(); void drawReleaseScreen();
void drawBattery4Bar(uint8_t bars);
void drawHeader(const char* label);
void drawCuffCircles();
void handleSerial(); void printHelp(); void printStatus();
void setupWiFi(); void setupServer();
void sendJSON(const char* json);
void debugValveCmd(String cmd);
void pressureStream(); void pressureTest();
void valveSweep(); void selfTest();

// ═══════════════════════════════════════════════════════════
//  EMBEDDED HTML UI (PROGMEM — no SPIFFS needed)
// ═══════════════════════════════════════════════════════════
const char HTML_UI[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>VerveFlo</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Rajdhani:wght@400;600;700&family=Share+Tech+Mono&display=swap');
:root{
  --bg:#070a0f;--s1:#0d1117;--s2:#111820;--border:#1a2535;
  --cyan:#00d4ff;--green:#00e5a0;--red:#ff3b5c;--amber:#ffb020;
  --purple:#8b5cf6;--text:#d8e4f0;--muted:#3d5068;
  --mono:'Share Tech Mono',monospace;--sans:'Rajdhani',sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:var(--bg);color:var(--text);font-family:var(--sans);min-height:100vh;overflow-x:hidden}
body::before{content:'';position:fixed;inset:0;
  background-image:linear-gradient(rgba(0,212,255,.02) 1px,transparent 1px),
  linear-gradient(90deg,rgba(0,212,255,.02) 1px,transparent 1px);
  background-size:36px 36px;pointer-events:none;z-index:0}
.app{position:relative;z-index:1;max-width:460px;margin:0 auto;padding:0 14px 40px}
header{display:flex;align-items:center;justify-content:space-between;padding:16px 0 12px;border-bottom:1px solid var(--border);margin-bottom:16px}
.logo{font-size:22px;font-weight:700;letter-spacing:2px;color:var(--cyan);text-shadow:0 0 20px rgba(0,212,255,.4)}
.logo span{color:var(--text);opacity:.6}
.pill{display:flex;align-items:center;gap:6px;padding:4px 10px;border-radius:20px;border:1px solid var(--border);font-family:var(--mono);font-size:10px;color:var(--muted);transition:all .3s}
.pill.live{border-color:rgba(0,229,160,.3);color:var(--green)}
.dot{width:6px;height:6px;border-radius:50%;background:var(--muted)}
.pill.live .dot{background:var(--green);box-shadow:0 0 6px var(--green);animation:blink 2s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}
.conn-row{display:flex;gap:8px;margin-bottom:14px}
.ip-in{flex:1;background:var(--s1);border:1px solid var(--border);border-radius:8px;color:var(--text);font-family:var(--mono);font-size:12px;padding:9px 12px;outline:none}
.ip-in:focus{border-color:var(--cyan)}
.ip-in::placeholder{color:var(--muted)}
.conn-btn{padding:0 14px;border-radius:8px;border:none;background:var(--cyan);color:#000;font-family:var(--sans);font-weight:700;font-size:13px;cursor:pointer}
.hero{background:var(--s1);border:1px solid var(--border);border-radius:16px;padding:18px;margin-bottom:12px}
.hero-top{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:14px}
.state-chip{display:inline-flex;align-items:center;gap:6px;padding:4px 12px;border-radius:6px;font-family:var(--mono);font-size:11px;border:1px solid;transition:all .3s}
.sr{color:var(--cyan);border-color:rgba(0,212,255,.3);background:rgba(0,212,255,.05)}
.sg{color:var(--green);border-color:rgba(0,229,160,.3);background:rgba(0,229,160,.05)}
.sa{color:var(--amber);border-color:rgba(255,176,32,.3);background:rgba(255,176,32,.05)}
.sx{color:var(--red);border-color:rgba(255,59,92,.3);background:rgba(255,59,92,.05)}
.batt-wrap{display:flex;flex-direction:column;align-items:flex-end;gap:3px}
.batt-icon{display:flex;align-items:center;gap:2px}
.batt-bar{width:8px;border-radius:2px;background:var(--border);transition:all .4s}
.batt-bar.filled{background:var(--green)}
.batt-bar.med{background:var(--amber)}
.batt-bar.low{background:var(--red)}
.batt-pct{font-family:var(--mono);font-size:9px;color:var(--muted)}
.metrics{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:14px}
.metric{display:flex;flex-direction:column;gap:2px}
.mlb{font-family:var(--mono);font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:1px}
.mvl{font-family:var(--mono);font-size:28px;color:var(--cyan);line-height:1}
.mvl.g{color:var(--green)}
.mun{font-size:11px;color:var(--muted);margin-left:2px}
.msb{font-family:var(--mono);font-size:10px;color:var(--muted);margin-top:2px}
.cuffs-lbl{font-family:var(--mono);font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:1px;margin-bottom:10px}
.cuffs-row{display:flex;gap:10px;align-items:center;justify-content:center;margin-bottom:6px}
.cuff-wrap{display:flex;flex-direction:column;align-items:center;gap:6px}
.cuff-circle{border-radius:50%;background:var(--border);border:2px solid transparent;transition:all .3s cubic-bezier(.4,0,.2,1);position:relative;overflow:hidden}
.cuff-circle.active{background:rgba(0,212,255,.15);border-color:var(--cyan);box-shadow:0 0 14px rgba(0,212,255,.5);animation:pump .8s ease-in-out infinite alternate}
@keyframes pump{from{transform:scale(1)}to{transform:scale(1.15)}}
.cuff-circle.rel{background:rgba(255,59,92,.15);border-color:var(--red);box-shadow:0 0 12px rgba(255,59,92,.4)}
.rel-lbl{font-family:var(--mono);font-size:8px;color:var(--muted)}
.pbar-wrap{margin-top:8px}
.pbar-lbl{font-family:var(--mono);font-size:8px;color:var(--muted);margin-bottom:4px;display:flex;justify-content:space-between}
.pbar-outer{height:4px;background:var(--border);border-radius:2px;position:relative}
.pbar-fill{height:100%;border-radius:2px;background:var(--cyan);transition:width .4s}
.pbar-tgt{position:absolute;top:-3px;width:2px;height:10px;background:var(--amber);border-radius:1px}
.main-btn{width:100%;height:64px;border-radius:14px;border:none;cursor:pointer;font-family:var(--sans);font-size:19px;font-weight:700;letter-spacing:.5px;position:relative;overflow:hidden;margin-bottom:10px;transition:all .2s}
.main-btn:active{transform:scale(.97)}
.bs{background:linear-gradient(135deg,#00d4ff,#0070e0);color:#000;box-shadow:0 0 24px rgba(0,212,255,.3)}
.bp{background:linear-gradient(135deg,#ffb020,#e06000);color:#000}
.br{background:linear-gradient(135deg,#00e5a0,#00879a);color:#000}
.bw{background:var(--s2);color:var(--muted);border:1px solid var(--border);cursor:not-allowed}
.sub-row{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:14px}
.sub-btn{padding:13px;border-radius:10px;border:none;cursor:pointer;font-family:var(--sans);font-size:13px;font-weight:600;transition:all .15s}
.sub-btn:active{transform:scale(.96)}
.bstop{background:rgba(255,59,92,.1);color:var(--red);border:1px solid rgba(255,59,92,.2)}
.brel{background:rgba(139,92,246,.1);color:var(--purple);border:1px solid rgba(139,92,246,.2)}
.sec{font-family:var(--mono);font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:2px;display:flex;align-items:center;gap:10px;margin:16px 0 10px}
.sec::after{content:'';flex:1;height:1px;background:var(--border)}
.card{background:var(--s1);border:1px solid var(--border);border-radius:12px;padding:14px;margin-bottom:10px}
.card-lbl{font-family:var(--mono);font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:1px;margin-bottom:10px}
.chips{display:flex;gap:6px;flex-wrap:wrap}
.chip{padding:6px 12px;border-radius:6px;font-family:var(--mono);font-size:11px;border:1px solid var(--border);background:transparent;color:var(--muted);cursor:pointer;transition:all .15s}
.chip.on{background:rgba(0,212,255,.1);border-color:rgba(0,212,255,.4);color:var(--cyan)}
.sl-row{display:flex;align-items:center;gap:12px;margin-bottom:10px}
.sl-lbl{font-family:var(--mono);font-size:10px;color:var(--muted);min-width:80px}
input[type=range]{flex:1;-webkit-appearance:none;height:3px;border-radius:2px;outline:none;background:linear-gradient(to right,var(--cyan) 0%,var(--cyan) var(--p,40%),var(--border) var(--p,40%))}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;border-radius:50%;background:var(--cyan);cursor:pointer}
.sl-val{font-family:var(--mono);font-size:12px;color:var(--text);min-width:30px;text-align:right}
.apl-btn{width:100%;padding:11px;border-radius:8px;border:none;background:rgba(0,212,255,.08);color:var(--cyan);border:1px solid rgba(0,212,255,.2);font-family:var(--sans);font-size:13px;font-weight:600;cursor:pointer;margin-top:10px}
.mtabs{display:flex;gap:6px;margin-bottom:12px;flex-wrap:wrap}
.mtab{padding:6px 12px;border-radius:6px;font-family:var(--mono);font-size:11px;border:1px solid var(--border);background:transparent;color:var(--muted);cursor:pointer;transition:all .15s}
.mtab.on{background:rgba(139,92,246,.12);border-color:rgba(139,92,246,.4);color:var(--purple)}
.mtab.ct{border-color:rgba(255,59,92,.3);color:rgba(255,59,92,.7)}
.mtab.ct.on{background:rgba(255,59,92,.1);color:var(--red)}
.mcard{background:var(--s1);border:1px solid var(--border);border-radius:12px;padding:16px}
.mname{font-size:14px;font-weight:600;margin-bottom:4px}
.mdesc{font-family:var(--mono);font-size:10px;color:var(--muted);margin-bottom:14px}
.seq-row{display:flex;align-items:center;gap:4px;flex-wrap:wrap;margin-bottom:14px}
.snode{display:flex;flex-direction:column;align-items:center;justify-content:center;width:38px;height:38px;border-radius:8px;border:1px solid var(--border);background:var(--s2);font-family:var(--mono)}
.snode.set{border-color:rgba(139,92,246,.5);background:rgba(139,92,246,.1)}
.snode .sn{font-size:14px;font-weight:700;color:var(--purple)}
.snode .sl{font-size:7px;color:var(--muted)}
.sarr{color:var(--muted);font-size:10px}
.pick-row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
.pick-c{width:44px;height:44px;border-radius:8px;background:var(--s2);border:1px solid var(--border);color:var(--muted);font-family:var(--mono);font-size:13px;font-weight:700;cursor:pointer;display:flex;align-items:center;justify-content:center}
.pick-c.used{opacity:.3;cursor:not-allowed}
.clr-c{width:44px;height:44px;border-radius:8px;background:rgba(255,59,92,.08);border:1px solid rgba(255,59,92,.2);color:var(--red);font-family:var(--mono);font-size:11px;cursor:pointer;display:flex;align-items:center;justify-content:center}
.save-c{width:100%;padding:12px;border-radius:8px;border:none;background:linear-gradient(135deg,rgba(255,59,92,.7),rgba(180,30,60,.7));color:#fff;font-family:var(--sans);font-size:14px;font-weight:700;cursor:pointer;margin-top:8px}
.use-btn{width:100%;padding:12px;border-radius:8px;border:none;background:rgba(139,92,246,.1);color:var(--purple);border:1px solid rgba(139,92,246,.3);font-family:var(--sans);font-size:14px;font-weight:700;cursor:pointer}
.setup-card{background:var(--s1);border:1px solid rgba(0,212,255,.2);border-radius:16px;padding:24px;margin:20px 0}
.setup-title{font-size:20px;font-weight:700;color:var(--cyan);margin-bottom:6px}
.setup-sub{font-family:var(--mono);font-size:11px;color:var(--muted);margin-bottom:20px}
.wifi-list{display:flex;flex-direction:column;gap:8px;margin-bottom:16px;max-height:240px;overflow-y:auto}
.wifi-item{padding:12px 14px;background:var(--s2);border:1px solid var(--border);border-radius:10px;cursor:pointer;display:flex;justify-content:space-between;align-items:center;transition:border-color .15s}
.wifi-item:hover{border-color:var(--cyan)}
.wifi-item.sel{border-color:var(--cyan);background:rgba(0,212,255,.05)}
.wifi-name{font-family:var(--mono);font-size:12px}
.wifi-rssi{font-family:var(--mono);font-size:10px;color:var(--muted)}
.pw-row{display:flex;gap:8px;margin-bottom:14px}
.pw-in{flex:1;background:var(--s2);border:1px solid var(--border);border-radius:8px;color:var(--text);font-family:var(--mono);font-size:13px;padding:10px 12px;outline:none}
.pw-in:focus{border-color:var(--cyan)}
.pw-in::placeholder{color:var(--muted)}
.conn-save{width:100%;padding:14px;border-radius:10px;border:none;background:linear-gradient(135deg,#00d4ff,#0091e6);color:#000;font-family:var(--sans);font-size:16px;font-weight:700;cursor:pointer}
.scan-btn{width:100%;padding:10px;border-radius:8px;border:1px solid var(--border);background:transparent;color:var(--muted);font-family:var(--mono);font-size:11px;cursor:pointer;margin-bottom:12px}
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%) translateY(60px);background:var(--s2);border:1px solid var(--border);border-radius:8px;padding:10px 18px;font-family:var(--mono);font-size:11px;color:var(--text);transition:transform .25s;z-index:100;white-space:nowrap;box-shadow:0 8px 32px rgba(0,0,0,.6)}
.toast.show{transform:translateX(-50%) translateY(0)}
</style>
</head>
<body>
<div class="app">
<header>
  <div class="logo">VERVE<span>FLO</span></div>
  <div class="pill" id="connPill"><div class="dot" id="connDot"></div><span id="connLbl">OFFLINE</span></div>
</header>
<div id="setupPage" style="display:none">
  <div class="setup-card">
    <div class="setup-title">WiFi Setup</div>
    <div class="setup-sub">Connect VerveFlo to your network</div>
    <button class="scan-btn" onclick="scanWifi()">⟳ Scan Networks</button>
    <div class="wifi-list" id="wifiList"></div>
    <div class="pw-row"><input class="pw-in" id="pwIn" type="password" placeholder="Password"></div>
    <button class="conn-save" onclick="saveWifi()">CONNECT & SAVE</button>
  </div>
</div>
<div id="therapyPage" style="display:none">
<div class="conn-row">
  <input class="ip-in" id="ipIn" type="text" placeholder="192.168.x.x" value="">
  <button class="conn-btn" onclick="doConnect()">CONNECT</button>
</div>
<div class="hero">
  <div class="hero-top">
    <div class="state-chip sr" id="stChip">● READY</div>
    <div class="batt-wrap">
      <div class="batt-icon" id="battIcon">
        <div class="batt-bar filled" style="height:14px"></div>
        <div class="batt-bar filled" style="height:11px"></div>
        <div class="batt-bar filled" style="height:8px"></div>
        <div class="batt-bar" style="height:6px"></div>
      </div>
      <div class="batt-pct" id="battPct">--%</div>
    </div>
  </div>
  <div class="metrics">
    <div class="metric">
      <div class="mlb">Pressure</div>
      <div class="mvl" id="pressVal">0<span class="mun">mmHg</span></div>
      <div class="msb" id="pressSub">Target: 70 mmHg</div>
    </div>
    <div class="metric">
      <div class="mlb">Time Left</div>
      <div class="mvl g" id="timeVal">--:--</div>
      <div class="msb" id="timeSub">10 min session</div>
    </div>
  </div>
  <div class="cuffs-lbl">Cuff Activity</div>
  <div class="cuffs-row">
    <div class="cuff-wrap"><div class="cuff-circle" id="c0" style="width:34px;height:34px"></div></div>
    <div class="cuff-wrap"><div class="cuff-circle" id="c1" style="width:30px;height:30px"></div></div>
    <div class="cuff-wrap"><div class="cuff-circle" id="c2" style="width:26px;height:26px"></div></div>
    <div class="cuff-wrap"><div class="cuff-circle" id="c3" style="width:30px;height:30px"></div></div>
    <div class="cuff-wrap"><div class="cuff-circle" id="c4" style="width:34px;height:34px"></div></div>
    <div style="width:1px;height:30px;background:var(--border);margin:0 4px"></div>
    <div class="cuff-wrap">
      <div class="cuff-circle" id="cR" style="width:22px;height:22px"></div>
      <div class="rel-lbl">REL</div>
    </div>
  </div>
  <div class="pbar-wrap">
    <div class="pbar-lbl"><span>0</span><span id="pbarLbl">0 mmHg</span><span>150</span></div>
    <div class="pbar-outer">
      <div class="pbar-fill" id="pbarFill" style="width:0%"></div>
      <div class="pbar-tgt" id="pbarTgt" style="left:46%"></div>
    </div>
  </div>
</div>
<button class="main-btn bs" id="mainBtn" onclick="mainAction()">▶  START SESSION</button>
<div class="sub-row">
  <button class="sub-btn bstop" onclick="doStop()">■  STOP</button>
  <button class="sub-btn brel"  onclick="doRelease()">⚡ RELEASE</button>
</div>
<div class="sec">Settings</div>
<div class="card">
  <div class="card-lbl">Pressure Target</div>
  <div class="chips" id="presChips">
    <button class="chip"    onclick="setPres(0,this)">LOW · 40mmHg</button>
    <button class="chip on" onclick="setPres(1,this)">MED · 70mmHg</button>
    <button class="chip"    onclick="setPres(2,this)">HIGH · 100mmHg</button>
  </div>
</div>
<div class="card">
  <div class="card-lbl">Session Duration</div>
  <div class="chips" id="timeChips">
    <button class="chip on" onclick="setTime(0,this)">10 min</button>
    <button class="chip"    onclick="setTime(1,this)">20 min</button>
    <button class="chip"    onclick="setTime(2,this)">30 min</button>
  </div>
</div>
<div class="card">
  <div class="card-lbl">Timings</div>
  <div class="sl-row">
    <span class="sl-lbl">Hold / cuff</span>
    <input type="range" id="holdS" min="3" max="30" value="8" oninput="sl(this,'holdV','s')">
    <span class="sl-val" id="holdV">8s</span>
  </div>
  <div class="sl-row" style="margin-bottom:0">
    <span class="sl-lbl">Deflate gap</span>
    <input type="range" id="gapS" min="1" max="15" value="2" oninput="sl(this,'gapV','s')">
    <span class="sl-val" id="gapV">2s</span>
  </div>
  <button class="apl-btn" onclick="applyTimings()">APPLY</button>
</div>
<div class="sec">Therapy Mode</div>
<div class="mtabs" id="mtabs">
  <button class="mtab on" onclick="pickMode(0,this)">M1</button>
  <button class="mtab"    onclick="pickMode(1,this)">M2</button>
  <button class="mtab"    onclick="pickMode(2,this)">M3</button>
  <button class="mtab"    onclick="pickMode(3,this)">M4</button>
  <button class="mtab ct" onclick="pickMode(4,this)">+ CUSTOM</button>
</div>
<div class="mcard">
  <div class="mname" id="mName">Mode 1 — Distal → Proximal</div>
  <div class="mdesc" id="mDesc">Standard venous return. Inflates foot→thigh sequentially.</div>
  <div class="seq-row" id="seqRow"></div>
  <div id="customUI" style="display:none">
    <div class="card-lbl" style="margin-bottom:10px">Tap cuffs in order</div>
    <div class="pick-row">
      <button class="pick-c" onclick="pickC(1,this)">C1</button>
      <button class="pick-c" onclick="pickC(2,this)">C2</button>
      <button class="pick-c" onclick="pickC(3,this)">C3</button>
      <button class="pick-c" onclick="pickC(4,this)">C4</button>
      <button class="pick-c" onclick="pickC(5,this)">C5</button>
      <button class="clr-c"  onclick="clrC()">CLR</button>
    </div>
    <div class="sl-row">
      <span class="sl-lbl">Repeat cycles</span>
      <input type="range" id="cycleS" min="1" max="10" value="3" oninput="sl(this,'cycleV','x')">
      <span class="sl-val" id="cycleV">3x</span>
    </div>
    <button class="save-c" onclick="saveCustom()">SAVE & USE</button>
  </div>
  <div id="presetUI">
    <button class="use-btn" onclick="useMode()">USE THIS MODE</button>
  </div>
</div>
</div>
</div>
<div class="toast" id="toast"></div>
<script>
const PRESETS=[
  {name:'Mode 1 — Distal → Proximal',desc:'Standard venous return. Inflates foot→thigh.',seq:[1,2,3,4,5]},
  {name:'Mode 2 — Proximal → Distal',desc:'Reverse drainage. Proximal to distal.',seq:[5,4,3,2,1]},
  {name:'Mode 3 — Alternating',desc:'Skip pattern. Targets deep vessel walls.',seq:[1,3,5,2,4]},
  {name:'Mode 4 — Wave',desc:'Complex wave for enhanced lymph drainage.',seq:[1,5,2,4,3]},
];
const PTARGETS=[40,70,100];
const TIMES=[10,20,30];
let S={connected:false,ip:'',sysState:'READY',pressure:0,activeCuff:-1,
  timeLeft:0,presIdx:1,timeIdx:0,mode:0,batt:100,battBars:4,
  customSeq:[],selTab:0,
  mockRun:false,mockPause:false,mockCuff:0,mockT:0};
let pollT,mockT,selSSID='';

window.onload=()=>{
  const isAP=window.location.hostname==='192.168.4.1'||window.location.search.includes('setup');
  if(isAP){document.getElementById('setupPage').style.display='block';scanWifi();}
  else{
    document.getElementById('therapyPage').style.display='block';
    if(window.location.hostname&&window.location.hostname!=='localhost'){
      document.getElementById('ipIn').value=window.location.hostname;
      doConnect();
    } else startMock();
  }
  renderSeq(PRESETS[0].seq);
  ['holdS','gapS','cycleS'].forEach(id=>{
    const el=document.getElementById(id);
    sl(el,id.replace('S','V'),id==='cycleS'?'x':'s');
  });
};
function scanWifi(){
  document.getElementById('wifiList').innerHTML='<div style="font-family:var(--mono);font-size:11px;color:var(--muted);padding:8px">Scanning...</div>';
  fetch('/scan').then(r=>r.json()).then(nets=>{
    const list=document.getElementById('wifiList');list.innerHTML='';
    nets.forEach(n=>{
      const d=document.createElement('div');d.className='wifi-item';
      d.innerHTML=`<span class="wifi-name">${n.ssid}</span><span class="wifi-rssi">${n.rssi}dBm</span>`;
      d.onclick=()=>{document.querySelectorAll('.wifi-item').forEach(x=>x.classList.remove('sel'));d.classList.add('sel');selSSID=n.ssid;};
      list.appendChild(d);
    });
  }).catch(()=>toast('Scan failed'));
}
function saveWifi(){
  if(!selSSID){toast('Select a network first');return}
  const pw=document.getElementById('pwIn').value;toast('Connecting...');
  fetch(`/savewifi?ssid=${encodeURIComponent(selSSID)}&pass=${encodeURIComponent(pw)}`)
    .then(r=>r.json()).then(d=>{if(d.ok)toast('Saved! Restarting...');else toast('Failed');})
    .catch(()=>toast('Error'));
}
function doConnect(){
  S.ip=document.getElementById('ipIn').value.trim()||window.location.hostname;
  toast('Connecting...');clearInterval(mockT);
  fetch(`http://${S.ip}/status`,{signal:AbortSignal.timeout(3000)})
    .then(r=>r.json()).then(d=>{S.connected=true;setConn(true);apply(d);startPoll();toast('✓ Connected');})
    .catch(()=>{setConn(false);toast('Demo mode');startMock();});
}
function setConn(v){
  S.connected=v;
  document.getElementById('connPill').classList.toggle('live',v);
  document.getElementById('connLbl').textContent=v?'LIVE':'OFFLINE';
}
function startPoll(){
  clearInterval(pollT);
  pollT=setInterval(()=>{
    fetch(`http://${S.ip}/status`,{signal:AbortSignal.timeout(2000)})
      .then(r=>r.json()).then(apply).catch(()=>setConn(false));
  },500);
}
function api(ep,params={}){
  if(!S.connected){mockCmd(ep);return}
  const qs=new URLSearchParams(params).toString();
  fetch(`http://${S.ip}/${ep}${qs?'?'+qs:''}`).catch(()=>{});
}
function apply(d){
  if(d.state)         S.sysState=d.state;
  if(d.pressure!=null)S.pressure=d.pressure;
  if(d.cuff!=null)    S.activeCuff=d.cuff;
  if(d.timeLeft!=null)S.timeLeft=d.timeLeft;
  if(d.mode!=null)    S.mode=d.mode;
  if(d.battery!=null) S.batt=d.battery;
  if(d.battBars!=null)S.battBars=d.battBars;
  if(d.presIdx!=null) S.presIdx=d.presIdx;
  if(d.timeIdx!=null) S.timeIdx=d.timeIdx;
  render();
}
function startMock(){
  clearInterval(mockT);
  mockT=setInterval(()=>{
    if(S.mockRun&&!S.mockPause){
      S.pressure=Math.min(92,S.pressure+(Math.random()*3-.4));
      if(S.pressure>90)S.pressure=22;
      S.mockT=Math.max(0,S.mockT-.4);S.timeLeft=Math.round(S.mockT);
      S.activeCuff=S.mockCuff;
      if(Math.random()<.04)S.mockCuff=(S.mockCuff+1)%5;
      S.sysState='RUNNING';
    }else if(!S.mockRun){S.pressure=0;S.activeCuff=-1;S.sysState='READY';}
    else S.sysState='PAUSED';
    render();
  },400);
}
function mockCmd(ep){
  if(ep==='start'){S.mockRun=true;S.mockPause=false;S.mockT=TIMES[S.timeIdx]*60;toast('▶ Started')}
  if(ep==='pause'){S.mockPause=!S.mockPause;toast(S.mockPause?'⏸ Paused':'▶ Resumed')}
  if(ep==='stop'||ep==='release'){S.mockRun=false;S.pressure=0;toast('■ Stopped')}
}
function render(){
  const st=S.sysState;
  const chip=document.getElementById('stChip');
  const sm={READY:{c:'sr',t:'● READY'},RUNNING:{c:'sg',t:'◆ RUNNING'},
    PAUSED:{c:'sa',t:'⏸ PAUSED'},RELEASING:{c:'sx',t:'↓ RELEASING'},
    DEFLATE_WAIT:{c:'sa',t:'↑ DEFLATING'}};
  const s=sm[st]||sm.READY;
  chip.className='state-chip '+s.c;chip.textContent=s.t;
  const bi=document.getElementById('battIcon');
  const heights=[14,11,8,6];bi.innerHTML='';
  for(let i=0;i<4;i++){
    const b=document.createElement('div');
    b.className='batt-bar'+(i<S.battBars?(S.battBars<=1?' low':S.battBars<=2?' med':' filled'):'');
    b.style.height=heights[i]+'px';bi.appendChild(b);
  }
  document.getElementById('battPct').textContent=S.batt+'%';
  document.getElementById('pressVal').innerHTML=Math.round(S.pressure)+'<span class="mun">mmHg</span>';
  document.getElementById('pressSub').textContent='Target: '+PTARGETS[S.presIdx]+' mmHg';
  const m=Math.floor(S.timeLeft/60),s2=S.timeLeft%60;
  document.getElementById('timeVal').textContent=st==='READY'?'--:--':`${String(m).padStart(2,'0')}:${String(s2).padStart(2,'0')}`;
  document.getElementById('timeSub').textContent=TIMES[S.timeIdx]+' min session';
  for(let i=0;i<5;i++){
    const c=document.getElementById('c'+i);if(!c)continue;
    c.className='cuff-circle'+(i===S.activeCuff&&(st==='RUNNING'||st==='DEFLATE_WAIT')?' active':'');
  }
  const cr=document.getElementById('cR');
  if(cr)cr.className='cuff-circle'+(st==='RELEASING'?' rel':'');
  const pct=Math.min(100,(S.pressure/150)*100);
  const tpct=(PTARGETS[S.presIdx]/150)*100;
  document.getElementById('pbarFill').style.width=pct+'%';
  document.getElementById('pbarTgt').style.left=tpct+'%';
  document.getElementById('pbarLbl').textContent=Math.round(S.pressure)+' mmHg';
  const btn=document.getElementById('mainBtn');
  if(st==='READY')       {btn.textContent='▶  START SESSION';btn.className='main-btn bs'}
  else if(st==='RUNNING'){btn.textContent='⏸  PAUSE';btn.className='main-btn bp'}
  else if(st==='PAUSED') {btn.textContent='▶  RESUME';btn.className='main-btn br'}
  else                   {btn.textContent='⏳  RELEASING...';btn.className='main-btn bw'}
}
function mainAction(){
  if(S.sysState==='READY')       api('start');
  else if(S.sysState==='RUNNING')api('pause');
  else if(S.sysState==='PAUSED') api('resume');
}
function doStop()   {api('stop')}
function doRelease(){api('release')}
function setPres(i,el){
  S.presIdx=i;
  document.querySelectorAll('#presChips .chip').forEach(c=>c.classList.remove('on'));
  el.classList.add('on');api('set_pressure',{level:i});
}
function setTime(i,el){
  S.timeIdx=i;
  document.querySelectorAll('#timeChips .chip').forEach(c=>c.classList.remove('on'));
  el.classList.add('on');api('set_time',{idx:i});
}
function applyTimings(){
  const h=document.getElementById('holdS').value;
  const g=document.getElementById('gapS').value;
  api('set_timings',{hold_ms:h*1000,gap_ms:g*1000});
  toast(`Hold ${h}s · Gap ${g}s applied`);
}
function sl(el,vId,unit){
  document.getElementById(vId).textContent=el.value+unit;
  el.style.setProperty('--p',((el.value-el.min)/(el.max-el.min)*100).toFixed(1)+'%');
}
function pickMode(idx,el){
  document.querySelectorAll('.mtab').forEach(t=>t.classList.remove('on'));
  el.classList.add('on');S.selTab=idx;
  const isC=idx===4;
  document.getElementById('customUI').style.display=isC?'block':'none';
  document.getElementById('presetUI').style.display=isC?'none':'block';
  if(!isC){const p=PRESETS[idx];document.getElementById('mName').textContent=p.name;document.getElementById('mDesc').textContent=p.desc;renderSeq(p.seq);}
  else{document.getElementById('mName').textContent='Custom Mode';document.getElementById('mDesc').textContent='Build your sequence below.';renderSeq(S.customSeq);}
}
function renderSeq(seq){
  const r=document.getElementById('seqRow');r.innerHTML='';
  for(let i=0;i<5;i++){
    if(i>0)r.innerHTML+=`<span class="sarr">→</span>`;
    const v=seq[i];
    r.innerHTML+=`<div class="snode ${v?'set':''}">
      ${v?`<span class="sn">C${v}</span><span class="sl">Step ${i+1}</span>`:`<span style="color:var(--muted);font-size:10px">?</span>`}
    </div>`;
  }
}
function pickC(n,btn){
  if(btn.classList.contains('used'))return;
  if(S.customSeq.length>=5){toast('Full — CLR first');return}
  S.customSeq.push(n);btn.classList.add('used');renderSeq(S.customSeq);
}
function clrC(){S.customSeq=[];document.querySelectorAll('.pick-c').forEach(b=>b.classList.remove('used'));renderSeq([]);toast('Cleared');}
function saveCustom(){
  if(S.customSeq.length<2){toast('Add at least 2 cuffs');return}
  api('set_custom_mode',{seq:S.customSeq.join(','),cycles:document.getElementById('cycleS').value});
  toast(`Custom: C${S.customSeq.join('→C')} saved`);
}
function useMode(){S.mode=S.selTab;api('set_mode',{mode:S.mode});toast(`Mode ${S.mode+1} activated`);}
let tT;
function toast(msg){
  const t=document.getElementById('toast');t.textContent=msg;t.classList.add('show');
  clearTimeout(tT);tT=setTimeout(()=>t.classList.remove('show'),2600);
}
</script>
</body>
</html>
)rawhtml";

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("\n╔══════════════════════════════╗"));
  Serial.println(F("║   VerveFlo v3.1  Booting...  ║"));
  Serial.println(F("╚══════════════════════════════╝"));

  pinMode(LED_PIN,  OUTPUT); digitalWrite(LED_PIN,  LOW);
  pinMode(PUMP_PIN, OUTPUT); analogWrite(PUMP_PIN,  0);

  // Valves — release valve starts HIGH (normally-open, so HIGH = closed)
  for (uint8_t i=0; i<6; i++) {
    pinMode(VALVE_PINS[i], OUTPUT);
    if (i == RELEASE_VALVE_IDX)
      analogWrite(VALVE_PINS[i], 255);  // VR: HIGH = closed (safe state)
    else
      analogWrite(VALVE_PINS[i], 0);    // Fill valves: LOW = closed
  }

  for (auto& b : g_btns) pinMode(b.pin, INPUT_PULLUP);
  pinMode(BATT_PIN, INPUT);

  readBattery();
  u8g2.begin();

  g_sysState     = STATE_BOOT;
  g_bootStep     = 0;
  g_bootStepTime = millis();
  setLed(LED_FAST);

  setupWiFi();
  setupServer();

  Serial.printf("[BOOT] Battery: %.2fV  %d%%  (%d bars)\n",
    g_battV, g_battPct, g_battBars);
  Serial.printf("[BOOT] CALIB_M=%.8f  CALIB_ZERO=%ld\n", CALIB_M, CALIB_ZERO);
  Serial.println(F("[BOOT] Type 'help' for commands\n"));
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  server.handleClient();
  if (g_apMode) dnsServer.processNextRequest();
  handleSerial();

  if (now - g_lastBattTime >= BATT_INTERVAL_MS) {
    readBattery(); g_lastBattTime = now;
  }

  // Boot sequence
  if (g_sysState == STATE_BOOT) {
    if (now - g_bootStepTime >= BOOT_STEP_MS) {
      g_bootStepTime = now; g_bootStep++;
      drawBootScreen();
      if (g_bootStep >= BOOT_STEPS) {
        g_sysState = STATE_READY;
        setLed(LED_SLOW);
        Serial.println(F("[BOOT] → READY"));
      }
    }
    updateLED(); return;
  }

  if (now - g_lastPressRead >= PRESS_MS) {
    g_pressure = readPressure();
    g_lastPressRead = now;
  }

  pollButtons();
  handleEvents();
  updateRamp();
  updateMotorRamp();

  if (g_sysState == STATE_RUNNING) updateSession();
  if (g_sysState == STATE_RELEASING || g_sysState == STATE_DEFLATE_WAIT)
    updateReleaseSequence();

  // Cuff animation tick
  if (now - g_lastAnim >= ANIM_MS) {
    g_lastAnim = now;
    for (uint8_t i=0; i<5; i++) {
      if ((int)i == MODE_SEQS[g_mode][g_cuffStep] &&
          (g_sysState==STATE_RUNNING || g_sysState==STATE_DEFLATE_WAIT))
        g_cuffAnim[i] = (g_cuffAnim[i]+1) % 8;
      else
        g_cuffAnim[i] = 0;
    }
  }

  if (now - g_lastDisplay >= DISPLAY_MS) {
    updateDisplay(); g_lastDisplay = now;
  }

  updateLED();
}

// ═══════════════════════════════════════════════════════════
//  WIFI SETUP
// ═══════════════════════════════════════════════════════════
void setupWiFi() {
  prefs.begin("verveflo", false);
  g_savedSSID = prefs.getString("ssid", "");
  g_savedPass = prefs.getString("pass", "");
  prefs.end();

  // If hardcoded credentials provided, save them
  if (strlen(WIFI_SSID) > 0 && g_savedSSID.length() == 0) {
    prefs.begin("verveflo", false);
    prefs.putString("ssid", WIFI_SSID);
    prefs.putString("pass", WIFI_PASS);
    prefs.end();
    g_savedSSID = WIFI_SSID;
    g_savedPass = WIFI_PASS;
  }

  if (g_savedSSID.length() > 0) {
    Serial.printf("[WIFI] Connecting to %s\n", g_savedSSID.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(g_savedSSID.c_str(), g_savedPass.c_str());
    uint8_t tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 24) {
      delay(500); Serial.print("."); tries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      g_wifiConnected = true;
      Serial.printf("\n[WIFI] Connected! Open: http://%s\n",
        WiFi.localIP().toString().c_str());
      return;
    }
    Serial.println(F("\n[WIFI] Failed → starting AP"));
  } else {
    Serial.println(F("[WIFI] No credentials → starting AP"));
  }

  // AP mode
  g_apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress apIP(192,168,4,1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
  dnsServer.start(DNS_PORT, "*", apIP);
  Serial.printf("[WIFI] AP: %s → open 192.168.4.1\n", AP_SSID);
}

// ═══════════════════════════════════════════════════════════
//  WEBSERVER
// ═══════════════════════════════════════════════════════════
void sendJSON(const char* json) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void setupServer() {
  server.on("/", HTTP_GET, [](){
    server.send_P(200, "text/html", HTML_UI);
  });

  // Captive portal redirects
  server.on("/generate_204",       [](){server.sendHeader("Location","http://192.168.4.1/");server.send(302);});
  server.on("/fwlink",             [](){server.sendHeader("Location","http://192.168.4.1/");server.send(302);});
  server.on("/hotspot-detect.html",[](){server.sendHeader("Location","http://192.168.4.1/");server.send(302);});

  server.on("/scan", HTTP_GET, [](){
    int n = WiFi.scanNetworks();
    String json = "[";
    for (int i=0; i<n; i++) {
      if (i>0) json += ",";
      json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + WiFi.RSSI(i) + "}";
    }
    json += "]";
    server.sendHeader("Access-Control-Allow-Origin","*");
    server.send(200,"application/json",json);
  });

  server.on("/savewifi", HTTP_GET, [](){
    if (server.hasArg("ssid") && server.hasArg("pass")) {
      prefs.begin("verveflo", false);
      prefs.putString("ssid", server.arg("ssid"));
      prefs.putString("pass", server.arg("pass"));
      prefs.end();
      sendJSON("{\"ok\":true}");
      Serial.printf("[WIFI] Saved: %s\n", server.arg("ssid").c_str());
      delay(1000); ESP.restart();
    } else sendJSON("{\"ok\":false}");
  });

  server.on("/wificlear", HTTP_GET, [](){
    prefs.begin("verveflo", false);
    prefs.remove("ssid"); prefs.remove("pass");
    prefs.end();
    sendJSON("{\"ok\":true}");
    delay(500); ESP.restart();
  });

  server.on("/status", HTTP_GET, [](){
    const char* stN[]={"BOOT","READY","RUNNING","PAUSED","RELEASING","DEFLATE_WAIT"};
    unsigned long rem=0;
    if (g_sysState==STATE_RUNNING||g_sysState==STATE_PAUSED||g_sysState==STATE_DEFLATE_WAIT){
      unsigned long el=millis()-g_sessionStartTime;
      if (el<g_sessionDurationMs) rem=(g_sessionDurationMs-el)/1000UL;
    }
    char json[320];
    snprintf(json,sizeof(json),
      "{\"state\":\"%s\",\"pressure\":%.1f,\"cuff\":%d,"
      "\"timeLeft\":%lu,\"mode\":%d,\"battery\":%d,\"battBars\":%d,"
      "\"presIdx\":%d,\"timeIdx\":%d,\"ok\":true}",
      stN[(int)g_sysState],g_pressure,g_cuffStep,
      rem,g_mode,g_battPct,g_battBars,g_pressureIdx,g_timeIdx);
    sendJSON(json);
  });

  server.on("/start",   [](){if(g_sysState==STATE_READY) startSession();  sendJSON("{\"ok\":true}");});
  server.on("/pause",   [](){
    if(g_sysState==STATE_RUNNING) pauseSession();
    else if(g_sysState==STATE_PAUSED) resumeSession();
    sendJSON("{\"ok\":true}");
  });
  server.on("/resume",  [](){if(g_sysState==STATE_PAUSED) resumeSession(); sendJSON("{\"ok\":true}");});
  server.on("/stop",    [](){beginRelease(); sendJSON("{\"ok\":true}");});
  server.on("/release", [](){beginRelease(); sendJSON("{\"ok\":true}");});

  server.on("/set_pressure",[](){
    if(server.hasArg("level")) g_pressureIdx=constrain(server.arg("level").toInt(),0,2);
    sendJSON("{\"ok\":true}");
  });
  server.on("/set_time",[](){
    if(server.hasArg("idx")) g_timeIdx=constrain(server.arg("idx").toInt(),0,2);
    sendJSON("{\"ok\":true}");
  });
  server.on("/set_mode",[](){
    if(server.hasArg("mode")) g_mode=constrain(server.arg("mode").toInt(),0,3);
    sendJSON("{\"ok\":true}");
  });
  server.on("/set_timings",[](){
    if(server.hasArg("hold_ms")) g_holdMs=constrain((uint32_t)server.arg("hold_ms").toInt(),2000UL,60000UL);
    if(server.hasArg("gap_ms"))  g_deflateWaitMs=constrain((uint32_t)server.arg("gap_ms").toInt(),1000UL,15000UL);
    sendJSON("{\"ok\":true}");
  });
  server.on("/set_custom_mode",[](){
    if(server.hasArg("seq"))
      Serial.printf("[CUSTOM] seq=%s\n", server.arg("seq").c_str());
    sendJSON("{\"ok\":true}");
  });

  server.onNotFound([](){
    if(g_apMode){ server.sendHeader("Location","http://192.168.4.1/"); server.send(302); }
    else server.send(404,"text/plain","Not found");
  });

  server.begin();
  Serial.println(F("[HTTP] Server started port 80"));
}

// ═══════════════════════════════════════════════════════════
//  BOOT SCREEN — SOLVIN logo → VerveFlo loading bar
// ═══════════════════════════════════════════════════════════
void drawBootScreen() {
  u8g2.clearBuffer();

  if (g_bootStep < 12) {
    // SOLVIN letter-by-letter reveal
    u8g2.setFont(u8g2_font_10x20_tr);
    const char* letters = "SOLVIN";
    uint8_t show = min((int)g_bootStep, 6);
    char buf[7] = {0};
    strncpy(buf, letters, show);
    uint8_t x = (128 - show * 10) / 2;
    u8g2.drawStr(x, 38, buf);
    if (g_bootStep >= 10) {
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(22, 52, "Medical Devices");
    }
  } else {
    // VerveFlo loading screen
    u8g2.setFont(u8g2_font_10x20_tr);
    u8g2.drawStr(14, 20, "VerveFlo");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(16, 30, "Compression Therapy");

    const char* msgs[] = {
      "Initialising...  ","Calibrating...   ",
      "Loading modes... ","Connecting WiFi. ","Ready.           "
    };
    uint8_t mi = (g_bootStep<16)?0:(g_bootStep<20)?1:(g_bootStep<23)?2:(g_bootStep<26)?3:4;
    u8g2.drawStr(4, 42, msgs[mi]);

    uint8_t barFill = (uint8_t)(((g_bootStep-12)*120UL)/(BOOT_STEPS-12));
    barFill = constrain(barFill, 0, 120);
    u8g2.drawFrame(4, 48, 120, 10);
    if (barFill > 0) u8g2.drawBox(5, 49, barFill, 8);

    u8g2.setFont(u8g2_font_4x6_tr);
    if (g_wifiConnected)
      u8g2.drawStr(4, 62, WiFi.localIP().toString().c_str());
    else if (g_apMode)
      u8g2.drawStr(4, 62, "AP: VerveFlo-Setup");
  }
  u8g2.sendBuffer();
}

// ═══════════════════════════════════════════════════════════
//  DISPLAY DISPATCHER
// ═══════════════════════════════════════════════════════════
void updateDisplay() {
  switch (g_sysState) {
    case STATE_READY:        drawReadyScreen();   break;
    case STATE_RUNNING:
    case STATE_DEFLATE_WAIT: drawRunScreen();     break;
    case STATE_PAUSED:       drawPausedScreen();  break;
    case STATE_RELEASING:    drawReleaseScreen(); break;
    default: break;
  }
}

void drawBattery4Bar(uint8_t bars) {
  u8g2.drawFrame(104, 1, 21, 10);
  u8g2.drawBox(125, 3, 2, 6);
  for (uint8_t i=0; i<4; i++)
    if (i < bars) u8g2.drawBox(106+i*5, 3, 4, 6);
}

void drawHeader(const char* label) {
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(0, 7, label);
  drawBattery4Bar(g_battBars);
  u8g2.drawHLine(0, 9, 128);
}

void drawCuffCircles() {
  const uint8_t cx[5]  = { 14, 32, 50, 68, 86 };
  const uint8_t cr2[5] = {  7,  6,  5,  6,  7 };
  for (uint8_t i=0; i<5; i++) {
    bool isAct = (MODE_SEQS[g_mode][g_cuffStep] == i &&
                  (g_sysState==STATE_RUNNING||g_sysState==STATE_DEFLATE_WAIT));
    if (isAct) {
      if (g_cuffAnim[i] < 4) u8g2.drawDisc(cx[i], 50, cr2[i]);
      else                    u8g2.drawCircle(cx[i], 50, cr2[i]);
    } else {
      u8g2.drawCircle(cx[i], 50, cr2[i]-1);
    }
  }
  if (g_sysState == STATE_RELEASING) u8g2.drawDisc(108, 50, 3);
  else                                u8g2.drawCircle(108, 50, 3);
}

void drawReadyScreen() {
  u8g2.clearBuffer();
  drawHeader("VERVEFLO");
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[24];
  snprintf(buf,sizeof(buf),"MODE %d  %s",g_mode+1,MODE_NAMES[g_mode]);
  u8g2.drawStr(0, 22, buf);
  snprintf(buf,sizeof(buf),"PRES  %s",PRESSURE_LABELS[g_pressureIdx]);
  u8g2.drawStr(0, 33, buf);
  snprintf(buf,sizeof(buf),"TIME  %s",TIME_LABELS[g_timeIdx]);
  u8g2.drawStr(0, 44, buf);
  u8g2.setFont(u8g2_font_4x6_tr);
  if (g_wifiConnected) u8g2.drawStr(0, 55, WiFi.localIP().toString().c_str());
  else if (g_apMode)   u8g2.drawStr(0, 55, "AP:VerveFlo-Setup");
  if ((millis()/600)%2==0) {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(20, 63, "[ PRESS START ]");
  }
  u8g2.sendBuffer();
}

void drawRunScreen() {
  u8g2.clearBuffer();
  drawHeader(g_sysState==STATE_DEFLATE_WAIT ? "DEFLATING" : "RUNNING");
  u8g2.setFont(u8g2_font_10x20_tr);
  char buf[16];
  snprintf(buf,sizeof(buf),"%3d",(int)g_pressure);
  u8g2.drawStr(0, 32, buf);
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(33, 24, "mmHg");
  snprintf(buf,sizeof(buf),"/%d",(int)PRESSURE_LEVELS[g_pressureIdx]);
  u8g2.drawStr(33, 32, buf);
  u8g2.drawStr(33, 40, PRESSURE_LABELS[g_pressureIdx]);
  unsigned long rem=0;
  if (millis()-g_sessionStartTime < g_sessionDurationMs)
    rem=(g_sessionDurationMs-(millis()-g_sessionStartTime))/1000UL;
  snprintf(buf,sizeof(buf),"%02lu:%02lu",rem/60,rem%60);
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(76, 28, buf);
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(80, 38, "remain");
  u8g2.drawStr(76, 44, MODE_NAMES[g_mode]);
  drawCuffCircles();
  uint8_t pFill = constrain((uint8_t)(g_pressure/150.0f*100),0,100);
  uint8_t tFill = constrain((uint8_t)(PRESSURE_LEVELS[g_pressureIdx]/150.0f*100),0,100);
  u8g2.drawFrame(0, 59, 102, 4);
  if (pFill>0) u8g2.drawBox(1, 60, pFill, 2);
  u8g2.drawLine(1+tFill, 58, 1+tFill, 63);
  u8g2.sendBuffer();
}

void drawPausedScreen() {
  u8g2.clearBuffer();
  drawHeader("PAUSED");
  if ((millis()/500)%2==0) {
    u8g2.setFont(u8g2_font_10x20_tr);
    u8g2.drawStr(16, 36, "PAUSED");
  }
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(4, 48, "Press START to resume");
  char buf[24];
  snprintf(buf,sizeof(buf),"M%d  %s  %s",
    g_mode+1,PRESSURE_LABELS[g_pressureIdx],TIME_LABELS[g_timeIdx]);
  u8g2.drawStr(20, 58, buf);
  u8g2.sendBuffer();
}

void drawReleaseScreen() {
  u8g2.clearBuffer();
  drawHeader("RELEASING");
  uint8_t prog = (g_releaseStep * 120) / NUM_INFLATE_VALVES;
  u8g2.drawFrame(4, 22, 120, 12);
  if (prog>0) u8g2.drawBox(5, 23, prog, 10);
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[24];
  snprintf(buf,sizeof(buf),"Cuff %d / %d",g_releaseStep+1,NUM_INFLATE_VALVES);
  u8g2.drawStr(22, 44, buf);
  drawCuffCircles();
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(18, 58, "Please wait...");
  u8g2.sendBuffer();
}

// ═══════════════════════════════════════════════════════════
//  PRESSURE READ
//  HX710B raw INCREASES with pressure — CALIB_M is positive
//  Overflow at ~16777216 during fast release — filtered
// ═══════════════════════════════════════════════════════════
float readPressure() {
  long raw = hx711.read();

  // Filter HX710B overflow value during fast pressure release
  if (raw > 16000000L || raw < 0) {
    return g_pressure;  // return last valid reading
  }

  long  adj    = raw - CALIB_ZERO;
  float mmhg   = CALIB_M * (float)adj + CALIB_B;
  return constrain(mmhg, 0.0f, 200.0f);
}

// ═══════════════════════════════════════════════════════════
//  VALVE CONTROL
//  Fill valves (0-4): 0=closed, 255=open   (normally-closed)
//  Release valve (5): 0=open,  255=closed  (normally-open, INVERTED)
// ═══════════════════════════════════════════════════════════
void startOpenValve(uint8_t idx) {
  bool isRel = (idx == RELEASE_VALVE_IDX);
  // Fill valve open  = ramp 0→255
  // Release open     = ramp 255→0 (de-energize to open)
  uint8_t startDuty = isRel ? 255 : 0;
  g_ramp = {true, true, startDuty, idx, millis()};
  analogWrite(VALVE_PINS[idx], startDuty);
  Serial.printf("[VALVE] Open V%d GPIO%d%s\n",
    idx+1, VALVE_PINS[idx], isRel?" [NO-INVERTED]":"");
}

void startCloseValve(uint8_t idx) {
  bool isRel = (idx == RELEASE_VALVE_IDX);
  // Fill valve close  = ramp 255→0
  // Release close     = ramp 0→255 (energize to close)
  uint8_t startDuty = isRel ? 0 : 255;
  g_ramp = {true, false, startDuty, idx, millis()};
  analogWrite(VALVE_PINS[idx], startDuty);
  Serial.printf("[VALVE] Close V%d GPIO%d%s\n",
    idx+1, VALVE_PINS[idx], isRel?" [NO-INVERTED]":"");
}

void updateRamp() {
  if (!g_ramp.active) return;
  unsigned long now = millis();
  if (now - g_ramp.lastStepTime < RAMP_STEP_INTERVAL) return;
  g_ramp.lastStepTime = now;

  bool isRel  = (g_ramp.valveIdx == RELEASE_VALVE_IDX);
  // XOR: invert ramp direction for release valve
  // opening=true  + fill    → ramp UP   (0→255)
  // opening=true  + release → ramp DOWN (255→0) = de-energize = open
  // opening=false + fill    → ramp DOWN (255→0)
  // opening=false + release → ramp UP   (0→255) = energize = close
  bool rampUp = g_ramp.opening ^ isRel;

  if (rampUp) {
    if (g_ramp.duty < 255) {
      analogWrite(VALVE_PINS[g_ramp.valveIdx], ++g_ramp.duty);
    } else {
      g_ramp.active = false;
      Serial.printf("[VALVE] Done V%d\n", g_ramp.valveIdx+1);
    }
  } else {
    if (g_ramp.duty > 0) {
      analogWrite(VALVE_PINS[g_ramp.valveIdx], --g_ramp.duty);
    } else {
      analogWrite(VALVE_PINS[g_ramp.valveIdx], 0);
      g_ramp.active = false;
      Serial.printf("[VALVE] Done V%d\n", g_ramp.valveIdx+1);
    }
  }
}

void allValvesOff() {
  g_ramp.active = false;
  for (uint8_t i=0; i<6; i++) {
    if (i == RELEASE_VALVE_IDX)
      analogWrite(VALVE_PINS[i], 255);  // Release: HIGH = closed (safe)
    else
      analogWrite(VALVE_PINS[i], 0);    // Fill: LOW = closed (safe)
  }
  Serial.println(F("[VALVE] All closed (safe state)"));
}

// ═══════════════════════════════════════════════════════════
//  PUMP — soft start via analogWrite ramp
// ═══════════════════════════════════════════════════════════
void setPump(bool on) {
  g_motorTarget  = on ? 255 : 0;
  g_motorRamping = true;
  g_motorRampT   = millis();
  Serial.printf("[PUMP] %s → ramping\n", on ? "ON" : "OFF");
}

void updateMotorRamp() {
  if (!g_motorRamping) return;
  unsigned long now = millis();
  if (now - g_motorRampT < 30) return;
  g_motorRampT = now;
  if (g_motorDuty < g_motorTarget)
    g_motorDuty = min((int)g_motorDuty + 5, (int)g_motorTarget);
  else if (g_motorDuty > g_motorTarget)
    g_motorDuty = max((int)g_motorDuty - 5, (int)g_motorTarget);
  else {
    g_motorRamping = false;
    Serial.printf("[PUMP] Ramp done duty=%d\n", g_motorDuty);
  }
  analogWrite(PUMP_PIN, g_motorDuty);
}

// ═══════════════════════════════════════════════════════════
//  SESSION CONTROL
// ═══════════════════════════════════════════════════════════
void advanceCuffStep() {
  uint8_t vi = MODE_SEQS[g_mode][g_cuffStep];
  startOpenValve(vi);
  g_cuffInflated = false;
  Serial.printf("[CUFF] Step %d → Cuff %d (V%d GPIO%d)\n",
    g_cuffStep+1, vi+1, vi+1, VALVE_PINS[vi]);
}

void startSession() {
  g_sessionStartTime  = millis();
  g_sessionDurationMs = (uint32_t)TIME_OPTIONS[g_timeIdx] * 60000UL;
  g_cuffStep          = 0;
  g_cuffInflated      = false;
  g_sysState          = STATE_RUNNING;
  setPump(true);
  advanceCuffStep();
  setLed(LED_PULSE);
  Serial.printf("[SESSION] START mode=%d pres=%s time=%s\n",
    g_mode+1, PRESSURE_LABELS[g_pressureIdx], TIME_LABELS[g_timeIdx]);
}

void pauseSession() {
  g_sysState = STATE_PAUSED;
  setPump(false);
  setLed(LED_SLOW);
  Serial.println(F("[SESSION] PAUSED"));
}

void resumeSession() {
  g_sysState = STATE_RUNNING;
  setPump(true);
  setLed(LED_PULSE);
  Serial.println(F("[SESSION] RESUMED"));
}

void beginRelease() {
  allValvesOff();
  setPump(false);
  g_releaseStep    = 0;
  g_releaseWaiting = false;
  g_sysState       = STATE_RELEASING;
  setLed(LED_FAST);
  Serial.println(F("[RELEASE] Started"));
}

void resetSystem() {
  allValvesOff();
  setPump(false);
  g_ramp.active  = false;
  g_cuffInflated = false;
  g_cuffStep     = 0;
  g_sysState     = STATE_READY;
  setLed(LED_SLOW);
  Serial.println(F("[SYSTEM] RESET → READY"));
}

void updateSession() {
  unsigned long now = millis();
  if (now - g_sessionStartTime >= g_sessionDurationMs) {
    Serial.println(F("[SESSION] Time complete"));
    beginRelease(); return;
  }
  float target = PRESSURE_LEVELS[g_pressureIdx];
  if (!g_cuffInflated) {
    if (g_pressure >= target) {
      if (g_ramp.active && g_ramp.opening) {
        g_ramp.active = false;
        // Hold fill valve fully open
        uint8_t vi = MODE_SEQS[g_mode][g_cuffStep];
        analogWrite(VALVE_PINS[vi], 255);
      }
      g_cuffInflated     = true;
      g_inflateHoldStart = now;
      Serial.printf("[CUFF] %.1fmmHg reached — holding %lums\n",
        g_pressure, g_holdMs);
    }
  } else if (now - g_inflateHoldStart >= g_holdMs) {
    startCloseValve(MODE_SEQS[g_mode][g_cuffStep]);
    startOpenValve(RELEASE_VALVE_IDX);
    g_cuffInflated     = false;
    g_sysState         = STATE_DEFLATE_WAIT;
    g_deflateWaitStart = now;
    Serial.printf("[CUFF] Deflating gap=%lums\n", g_deflateWaitMs);
  }
}

void updateReleaseSequence() {
  unsigned long now = millis();
  if (g_sysState == STATE_DEFLATE_WAIT) {
    if (now - g_deflateWaitStart >= g_deflateWaitMs) {
      startCloseValve(RELEASE_VALVE_IDX);
      g_cuffStep = (g_cuffStep + 1) % NUM_INFLATE_VALVES;
      g_sysState = STATE_RUNNING;
      advanceCuffStep();
    }
    return;
  }
  if (g_releaseStep >= NUM_INFLATE_VALVES) {
    startCloseValve(RELEASE_VALVE_IDX);
    Serial.println(F("[RELEASE] Complete"));
    resetSystem(); return;
  }
  if (!g_releaseWaiting) {
    // Close this inflate cuff (fill valve LOW = closed)
    analogWrite(VALVE_PINS[MODE_SEQS[g_mode][g_releaseStep]], 0);
    startOpenValve(RELEASE_VALVE_IDX);  // open release valve to vent
    g_releaseStepStart = now;
    g_releaseWaiting   = true;
    Serial.printf("[RELEASE] Cuff %d\n", g_releaseStep+1);
  } else if (now - g_releaseStepStart >= RELEASE_STEP_WAIT_MS) {
    startCloseValve(RELEASE_VALVE_IDX);
    g_releaseStep++;
    g_releaseWaiting = false;
  }
}

// ═══════════════════════════════════════════════════════════
//  BUTTON POLLING
// ═══════════════════════════════════════════════════════════
void pollButtons() {
  unsigned long now = millis();
  for (uint8_t i=0; i<4; i++) {
    Button& b = g_btns[i];
    bool raw = digitalRead(b.pin);
    if (raw != b.lastRaw) { b.lastChangeTime = now; b.lastRaw = raw; }
    if (now - b.lastChangeTime >= DEBOUNCE_MS) {
      if (raw != b.state) {
        b.state = raw;
        if (raw == LOW) { b.pressTime = now; b.longFired = false; }
        else if (!b.longFired) {
          switch(i){
            case 0: g_evt_start    = true; break;
            case 1: g_evt_mode     = true; break;
            case 2: g_evt_pressure = true; break;
            case 3: g_evt_time     = true; break;
          }
        }
      }
      if (i==0 && b.state==LOW && !b.longFired &&
          (now - b.pressTime) >= LONGPRESS_MS) {
        b.longFired = true;
        g_evt_reset = true;
        g_evt_start = false;
        Serial.println(F("[BTN] Long press → RESET"));
      }
    }
  }
}

void handleEvents() {
  if (g_evt_reset) { g_evt_reset = g_evt_start = false; beginRelease(); return; }
  if (g_evt_start) {
    g_evt_start = false;
    if      (g_sysState==STATE_READY)   startSession();
    else if (g_sysState==STATE_RUNNING) pauseSession();
    else if (g_sysState==STATE_PAUSED)  resumeSession();
  }
  if (g_evt_pressure && (g_sysState==STATE_READY||g_sysState==STATE_PAUSED)) {
    g_pressureIdx = (g_pressureIdx+1)%3;
    Serial.printf("[BTN] Pressure → %s\n", PRESSURE_LABELS[g_pressureIdx]);
  }
  if (g_evt_time && (g_sysState==STATE_READY||g_sysState==STATE_PAUSED)) {
    g_timeIdx = (g_timeIdx+1)%3;
    Serial.printf("[BTN] Time → %s\n", TIME_LABELS[g_timeIdx]);
  }
  if (g_evt_mode && (g_sysState==STATE_READY||g_sysState==STATE_PAUSED)) {
    g_mode = (g_mode+1)%4;
    Serial.printf("[BTN] Mode → %d (%s)\n", g_mode+1, MODE_NAMES[g_mode]);
  }
  g_evt_pressure = g_evt_time = g_evt_mode = false;
}

// ═══════════════════════════════════════════════════════════
//  SERIAL COMMANDS
// ═══════════════════════════════════════════════════════════
void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); cmd.toLowerCase();

  if(cmd=="help")    { printHelp(); return; }
  if(cmd=="status")  { printStatus(); return; }

  if(cmd=="start") {
    if(g_sysState==STATE_READY) startSession();
    else Serial.println(F("Not in READY state"));
    return;
  }
  if(cmd=="pause") {
    if      (g_sysState==STATE_RUNNING) pauseSession();
    else if (g_sysState==STATE_PAUSED)  resumeSession();
    else Serial.println(F("Not running/paused"));
    return;
  }
  if(cmd=="stop"||cmd=="release") { beginRelease(); return; }

  if(cmd=="pressure") {
    Serial.printf("[PRESSURE] %.2f mmHg  raw adj=~%ld\n",
      g_pressure, (long)(g_pressure/CALIB_M) + CALIB_ZERO);
    return;
  }
  if(cmd=="zero") {
    Serial.println(F("[ZERO] Sampling 16 reads at atmosphere..."));
    long sum = 0;
    for(uint8_t i=0; i<16; i++) { sum += hx711.read(); delay(10); }
    CALIB_ZERO = sum / 16;
    Serial.printf("[ZERO] ✓ Set to %ld\n", CALIB_ZERO);
    Serial.println(F("[ZERO] Pressure should now read ~0 mmHg"));
    return;
  }
  if(cmd=="battery") {
    Serial.printf("[BATTERY] %.2fV  %d%%  %d/4 bars\n",
      g_battV, g_battPct, g_battBars);
    return;
  }
  if(cmd=="ip") {
    if(g_wifiConnected) Serial.printf("[WIFI] http://%s\n",
      WiFi.localIP().toString().c_str());
    else if(g_apMode) Serial.printf("[WIFI] AP http://192.168.4.1  (%s)\n", AP_SSID);
    else Serial.println(F("[WIFI] Not connected"));
    return;
  }
  if(cmd=="valves") {
    Serial.println(F("[VALVES]"));
    const char* vType[] = {"FILL(NC)","FILL(NC)","FILL(NC)","FILL(NC)","FILL(NC)","REL(NO)"};
    for(uint8_t i=0; i<6; i++)
      Serial.printf("  V%d GPIO%2d [%s]: %s\n", i+1, VALVE_PINS[i], vType[i],
        (g_ramp.active&&g_ramp.valveIdx==i)?(g_ramp.opening?"RAMPING OPEN":"RAMPING CLOSE"):"IDLE");
    return;
  }
  if(cmd=="buttons") {
    Serial.println(F("[BUTTONS]"));
    Serial.printf("  START  GPIO35: %s\n",digitalRead(BTN_START)?"HI":"LO (PRESSED)");
    Serial.printf("  MODE   GPIO36: %s\n",digitalRead(BTN_MODE) ?"HI":"LO (PRESSED)");
    Serial.printf("  PRESS  GPIO37: %s\n",digitalRead(BTN_PRESS)?"HI":"LO (PRESSED)");
    Serial.printf("  TIME   GPIO48: %s\n",digitalRead(BTN_TIME) ?"HI":"LO (PRESSED)");
    return;
  }
  if(cmd=="pins") {
    Serial.println(F("[PINS]"));
    Serial.printf("  LED   GPIO 3 : %s\n",digitalRead(LED_PIN)?"HI":"LO");
    Serial.printf("  PUMP  GPIO21 : duty=%d\n", g_motorDuty);
    Serial.printf("  V1    GPIO26 : %s\n",digitalRead(VALVE_PINS[0])?"HI":"LO");
    Serial.printf("  V2    GPIO47 : %s\n",digitalRead(VALVE_PINS[1])?"HI":"LO");
    Serial.printf("  V3    GPIO39 : %s\n",digitalRead(VALVE_PINS[2])?"HI":"LO");
    Serial.printf("  V4    GPIO40 : %s\n",digitalRead(VALVE_PINS[3])?"HI":"LO");
    Serial.printf("  V5    GPIO41 : %s\n",digitalRead(VALVE_PINS[4])?"HI":"LO");
    Serial.printf("  VR    GPIO42 : %s (NO: HI=closed LO=open)\n",
      digitalRead(VALVE_PINS[5])?"HI":"LO");
    return;
  }
  if(cmd=="pump on")  { setPump(true);  return; }
  if(cmd=="pump off") { setPump(false); return; }
  if(cmd=="alloff")   { allValvesOff(); setPump(false); Serial.println(F("[ALLOFF] Done")); return; }

  if(cmd.startsWith("motor ")) {
    int d = constrain(cmd.substring(6).toInt(), 0, 255);
    g_motorTarget=d; g_motorRamping=true; g_motorRampT=millis();
    Serial.printf("[MOTOR] Target=%d\n", d);
    return;
  }

  if(cmd.startsWith("v")) { debugValveCmd(cmd); return; }

  if(cmd=="pstream") { pressureStream(); return; }
  if(cmd=="ptest")   { pressureTest();   return; }
  if(cmd=="vsweep")  { valveSweep();     return; }
  if(cmd=="selftest"){ selfTest();       return; }

  if(cmd=="led on")  { g_ledForced=true;  digitalWrite(LED_PIN,HIGH); Serial.println(F("[LED] ON")); return; }
  if(cmd=="led off") { g_ledForced=true;  digitalWrite(LED_PIN,LOW);  Serial.println(F("[LED] OFF")); return; }
  if(cmd=="led blink"){
    Serial.println(F("[LED] Blink 5x"));
    for(uint8_t i=0;i<10;i++){digitalWrite(LED_PIN,i%2==0);delay(200);}
    digitalWrite(LED_PIN,LOW); return;
  }
  if(cmd=="led auto"){ g_ledForced=false; Serial.println(F("[LED] Auto")); return; }

  if(cmd=="wificlear") {
    prefs.begin("verveflo",false); prefs.remove("ssid"); prefs.remove("pass"); prefs.end();
    Serial.println(F("[WIFI] Cleared → restarting")); delay(500); ESP.restart(); return;
  }
  if(cmd.startsWith("mode ")) {
    int m=cmd.substring(5).toInt();
    if(m>=1&&m<=4&&(g_sysState==STATE_READY||g_sysState==STATE_PAUSED))
      { g_mode=m-1; Serial.printf("[MODE] %d (%s)\n",g_mode+1,MODE_NAMES[g_mode]); }
    else Serial.println(F("mode 1-4  (only in READY/PAUSED)"));
    return;
  }
  if(cmd.startsWith("pset ")) {
    int p=cmd.substring(5).toInt();
    if(p>=0&&p<=2) { g_pressureIdx=p; Serial.printf("[PRES] %s\n",PRESSURE_LABELS[p]); }
    else Serial.println(F("pset 0/1/2"));
    return;
  }
  if(cmd.startsWith("time ")) {
    int t=cmd.substring(5).toInt();
    if(t>=0&&t<=2&&(g_sysState==STATE_READY||g_sysState==STATE_PAUSED))
      { g_timeIdx=t; Serial.printf("[TIME] %s\n",TIME_LABELS[t]); }
    else Serial.println(F("time 0/1/2"));
    return;
  }
  if(cmd.startsWith("hold ")) {
    int h=cmd.substring(5).toInt();
    if(h>=2&&h<=60) { g_holdMs=(uint32_t)h*1000UL; Serial.printf("[HOLD] %ds\n",h); }
    else Serial.println(F("hold 2-60"));
    return;
  }
  if(cmd.startsWith("gap ")) {
    int g=cmd.substring(4).toInt();
    if(g>=1&&g<=15) { g_deflateWaitMs=(uint32_t)g*1000UL; Serial.printf("[GAP] %ds\n",g); }
    else Serial.println(F("gap 1-15"));
    return;
  }
  if(cmd=="reboot") {
    Serial.println(F("[SYSTEM] Rebooting...")); delay(200); ESP.restart(); return;
  }
  Serial.printf("Unknown: '%s' — type 'help'\n", cmd.c_str());
}

void debugValveCmd(String cmd) {
  int8_t vIdx = -1;
  String rest = "";
  if(cmd.startsWith("vr ")) {
    vIdx = RELEASE_VALVE_IDX;
    rest = cmd.substring(3); rest.trim();
  } else if(cmd.length()>=3 && cmd[1]>='1' && cmd[1]<='5') {
    vIdx = cmd[1]-'1';
    rest = cmd.substring(2); rest.trim();
  }
  if(vIdx < 0) { Serial.println(F("v1-v5 on/off/open/close/0-255  vr on/off")); return; }

  bool isRel = (vIdx == RELEASE_VALVE_IDX);

  if(rest=="open")       { startOpenValve(vIdx); }
  else if(rest=="close") { startCloseValve(vIdx); }
  else if(rest=="on") {
    g_ramp.active = false;
    // VR: on = de-energize (LOW) = open. Fill: on = HIGH = open
    uint8_t duty = isRel ? 0 : 255;
    analogWrite(VALVE_PINS[vIdx], duty);
    Serial.printf("[V%d] GPIO%d ON (duty=%d)%s\n",
      vIdx+1, VALVE_PINS[vIdx], duty, isRel?" [NO: de-energized=open]":"");
  }
  else if(rest=="off") {
    g_ramp.active = false;
    // VR: off = energize (HIGH) = closed. Fill: off = LOW = closed
    uint8_t duty = isRel ? 255 : 0;
    analogWrite(VALVE_PINS[vIdx], duty);
    Serial.printf("[V%d] GPIO%d OFF (duty=%d)%s\n",
      vIdx+1, VALVE_PINS[vIdx], duty, isRel?" [NO: energized=closed]":"");
  }
  else {
    int d = rest.toInt();
    if(d>=0 && d<=255) {
      g_ramp.active = false;
      analogWrite(VALVE_PINS[vIdx], d);
      Serial.printf("[V%d] GPIO%d duty=%d\n", vIdx+1, VALVE_PINS[vIdx], d);
    } else Serial.println(F("Options: on/off/open/close/0-255"));
  }
}

void pressureStream() {
  Serial.println(F("[PSTREAM] Any key to stop"));
  Serial.println(F("─────────────────────────────────────────────"));
  while(true) {
    if(Serial.available()) { Serial.read(); break; }
    long raw = hx711.read();
    if(raw > 16000000L || raw < 0) { delay(200); continue; }
    long  adj  = raw - CALIB_ZERO;
    float mmhg = constrain(CALIB_M*(float)adj+CALIB_B, 0.0f, 200.0f);
    uint8_t bars = constrain((uint8_t)(mmhg/4.0f), 0, 30);
    char bar[32];
    for(uint8_t i=0;i<30;i++) bar[i]=(i<bars)?'#':'.'; bar[30]='\0';
    Serial.printf("  %6.1f mmHg  |%s|\n", mmhg, bar);
    delay(250);
  }
  Serial.println(F("[PSTREAM] Stopped"));
}

void pressureTest() {
  float target = PRESSURE_LEVELS[g_pressureIdx];
  Serial.printf("[PTEST] Target %.0f mmHg — any key to abort\n", target);
  setPump(true);
  unsigned long start = millis();
  while(true) {
    if(Serial.available()) { Serial.read(); Serial.println(F("[PTEST] Aborted")); break; }
    long raw = hx711.read();
    if(raw > 16000000L || raw < 0) { delay(200); continue; }
    float p = constrain(CALIB_M*(float)(raw-CALIB_ZERO)+CALIB_B, 0.0f, 200.0f);
    uint8_t bars = constrain((uint8_t)(p/4.0f), 0, 30);
    char bar[32]; for(uint8_t i=0;i<30;i++) bar[i]=(i<bars)?'#':'.'; bar[30]='\0';
    Serial.printf("  %6.1f / %.0f mmHg  |%s|\n", p, target, bar);
    if(p >= target) { Serial.printf("[PTEST] ✓ Reached in %lums\n",millis()-start); break; }
    if(millis()-start > 30000) { Serial.println(F("[PTEST] Timeout 30s")); break; }
    delay(250);
  }
  setPump(false);
}

void valveSweep() {
  Serial.println(F("[VSWEEP] Each inflate valve 2s — any key to stop"));
  allValvesOff();
  for(uint8_t i=0; i<5; i++) {
    if(Serial.available()) { Serial.read(); break; }
    Serial.printf("[VSWEEP] V%d GPIO%d ON\n", i+1, VALVE_PINS[i]);
    analogWrite(VALVE_PINS[i], 255);
    unsigned long t = millis();
    while(millis()-t < 2000) { if(Serial.available()) goto sweep_done; delay(50); }
    analogWrite(VALVE_PINS[i], 0);
    Serial.printf("[VSWEEP] V%d OFF\n", i+1);
    delay(200);
  }
  sweep_done:
  allValvesOff();
  Serial.println(F("[VSWEEP] Done"));
}

void selfTest() {
  Serial.println(F("\n══ VerveFlo Self Test v3.1 ══════════════════"));
  Serial.println(F("[1/6] LED GPIO3"));
  for(uint8_t i=0;i<6;i++){digitalWrite(LED_PIN,i%2==0);delay(250);}
  digitalWrite(LED_PIN,LOW);
  Serial.println(F("      ✓ Should have blinked 3x"));

  Serial.println(F("[2/6] Battery GPIO15"));
  readBattery();
  Serial.printf("      %.2fV  %d%%  %d/4 bars  %s\n",
    g_battV, g_battPct, g_battBars,
    g_battPct>10?"✓":"✗ Check divider");

  Serial.println(F("[3/6] Pressure sensor DATA=GPIO34 CLK=GPIO33"));
  long raw = hx711.read();
  float mmhg = (raw > 16000000L || raw < 0) ? -1.0f :
    constrain(CALIB_M*(float)(raw-CALIB_ZERO)+CALIB_B, 0.0f, 200.0f);
  Serial.printf("      raw=%ld  mmhg=%.2f  %s\n", raw, mmhg,
    (raw!=0&&raw<16000000L)?"✓":"✗ Check wiring");

  Serial.println(F("[4/6] Pump motor GPIO21 (2s soft start)"));
  setPump(true); delay(2500); setPump(false); delay(2000);
  Serial.println(F("      ✓ Done"));

  Serial.println(F("[5/6] Fill valves (1s each)  [NC: HIGH=open]"));
  const uint8_t gpios[]={26,47,39,40,41};
  for(uint8_t i=0;i<5;i++){
    Serial.printf("      V%d GPIO%d ON\n",i+1,gpios[i]);
    analogWrite(VALVE_PINS[i],255); delay(1000);
    analogWrite(VALVE_PINS[i],0);   delay(200);
  }
  Serial.println(F("      ✓ Done"));

  Serial.println(F("[6/6] Release valve GPIO42  [NO: LOW=open HIGH=closed]"));
  Serial.println(F("      Opening (de-energize)..."));
  analogWrite(VALVE_PINS[5], 0);   delay(1000);
  Serial.println(F("      Closing (energize)..."));
  analogWrite(VALVE_PINS[5], 255); delay(500);
  Serial.println(F("      ✓ Done"));

  Serial.println(F("\n[INFO] Buttons:"));
  Serial.printf("  START GPIO35:%s  MODE GPIO36:%s  PRES GPIO37:%s  TIME GPIO48:%s\n",
    digitalRead(BTN_START)?"HI":"LO",
    digitalRead(BTN_MODE) ?"HI":"LO",
    digitalRead(BTN_PRESS)?"HI":"LO",
    digitalRead(BTN_TIME) ?"HI":"LO");

  if(g_wifiConnected)
    Serial.printf("[INFO] WiFi: http://%s\n", WiFi.localIP().toString().c_str());
  else if(g_apMode)
    Serial.println(F("[INFO] AP: http://192.168.4.1"));

  Serial.println(F("══ Self Test Complete ════════════════════════\n"));
}

void printHelp() {
  Serial.println(F("\n═══ VerveFlo v3.1 Commands ══════════════════════"));
  Serial.println(F("  SESSION  : start / pause / stop / release"));
  Serial.println(F("  INFO     : status / pressure / battery / valves"));
  Serial.println(F("           : buttons / pins / ip"));
  Serial.println(F("  CALIBRATE: zero  (set pressure zero at atmosphere)"));
  Serial.println(F("  VALVES   : v1-v5 on/off/open/close/0-255"));
  Serial.println(F("           : vr on/off  (NO valve: on=open off=closed)"));
  Serial.println(F("  PUMP     : pump on / pump off / motor 0-255"));
  Serial.println(F("  TESTS    : pstream / ptest / vsweep / selftest / alloff"));
  Serial.println(F("  LED      : led on/off/blink/auto"));
  Serial.println(F("  SETTINGS : mode 1-4 / pset 0-2 / time 0-2"));
  Serial.println(F("           : hold N(sec) / gap N(sec)"));
  Serial.println(F("  WIFI     : wificlear / ip"));
  Serial.println(F("  SYSTEM   : reboot / help"));
  Serial.println(F("═════════════════════════════════════════════════\n"));
}

void printStatus() {
  const char* stN[]={"BOOT","READY","RUNNING","PAUSED","RELEASING","DEFLATE_WAIT"};
  Serial.println(F("\n─── VerveFlo v3.1 Status ─────────────────────────"));
  Serial.printf("  State      : %s\n",   stN[(int)g_sysState]);
  Serial.printf("  Mode       : %d (%s)\n", g_mode+1, MODE_NAMES[g_mode]);
  Serial.printf("  Pressure   : %.2f mmHg  target=%s/%.0f\n",
    g_pressure, PRESSURE_LABELS[g_pressureIdx], PRESSURE_LEVELS[g_pressureIdx]);
  Serial.printf("  CALIB_M    : %.8f\n",  CALIB_M);
  Serial.printf("  CALIB_ZERO : %ld\n",   CALIB_ZERO);
  Serial.printf("  Battery    : %.2fV  %d%%  %d/4 bars\n",
    g_battV, g_battPct, g_battBars);
  Serial.printf("  Session    : %s  hold=%lus  gap=%lus\n",
    TIME_LABELS[g_timeIdx], g_holdMs/1000, g_deflateWaitMs/1000);
  Serial.printf("  Cuff step  : %d/5\n",  g_cuffStep+1);
  Serial.printf("  Motor duty : %d  ramping=%s\n",
    g_motorDuty, g_motorRamping?"YES":"NO");
  Serial.printf("  Valve ramp : %s V%d %s duty=%d\n",
    g_ramp.active?"ACTIVE":"IDLE",
    g_ramp.valveIdx+1,
    g_ramp.opening?"OPEN":"CLOSE",
    g_ramp.duty);
  if(g_wifiConnected)
    Serial.printf("  WiFi       : http://%s\n", WiFi.localIP().toString().c_str());
  else if(g_apMode)
    Serial.printf("  AP Mode    : http://192.168.4.1 (%s)\n", AP_SSID);
  if(g_sysState==STATE_RUNNING||g_sysState==STATE_PAUSED||g_sysState==STATE_DEFLATE_WAIT){
    unsigned long rem=(g_sessionDurationMs-(millis()-g_sessionStartTime))/1000UL;
    Serial.printf("  Time left  : %lum %lus\n", rem/60, rem%60);
  }
  Serial.println(F("──────────────────────────────────────────────────\n"));
}
