/*
 * VerveFlo — Valve Test (with OLED)
 * ESP32-S3 Dev Module · Arduino IDE
 *
 * LIBRARY: U8g2 by olikraus (Tools -> Manage Libraries)
 *
 * BOARD: ESP32S3 Dev Module · USB CDC On Boot: Enabled
 * SERIAL: 115200 baud, Newline
 *
 * Two-button interface:
 *   BTN Pressure (GPIO37) -> cycle which valve is active
 *   BTN Time     (GPIO48) -> cycle dwell time (500/1000/1500/3000/5000 ms)
 *
 * Serial commands also work:
 *   v1..v5 on/off · vr on/off · auto (auto-cycle) · stop · alloff
 *
 * OLED shows: active valve, dwell time, and live valve state.
 */

#include <U8g2lib.h>
#include <Wire.h>

U8g2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Index: 0=Fill1, 1=Fill2, 2=Fill3, 3=Fill4, 4=Fill5, 5=Release
static const uint8_t VALVE_PINS[6] = { 26, 47, 39, 40, 41, 42 };
const char* VALVE_NAME[6] = { "Fill1", "Fill2", "Fill3", "Fill4", "Fill5", "Release" };

#define BTN_PRESS  37   // cycle valve
#define BTN_TIME   48   // cycle dwell
#define LED_PIN    3

const uint16_t DWELLS[5] = { 500, 1000, 1500, 3000, 5000 };

int      g_valve   = 0;     // selected valve index
int      g_dwellIx = 2;     // -> 1500 ms
bool     g_auto    = false;
uint32_t g_lastStep = 0;
bool     g_valveOn  = false;

bool g_pPress = HIGH, g_pTime = HIGH;   // previous button states

void valveWrite(int idx, bool on) {
  for (int i = 0; i < 6; i++) digitalWrite(VALVE_PINS[i], LOW);
  if (on && idx >= 0) digitalWrite(VALVE_PINS[idx], HIGH);
  g_valveOn = on;
  digitalWrite(LED_PIN, on);
}

void draw() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 10, "VerveFlo Valve Test");
  u8g2.drawHLine(0, 13, 128);

  char buf[24];
  snprintf(buf, sizeof(buf), "Valve: %s", VALVE_NAME[g_valve]);
  u8g2.drawStr(0, 30, buf);
  snprintf(buf, sizeof(buf), "GPIO : %d", VALVE_PINS[g_valve]);
  u8g2.drawStr(0, 42, buf);
  snprintf(buf, sizeof(buf), "Dwell: %d ms", DWELLS[g_dwellIx]);
  u8g2.drawStr(0, 54, buf);

  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(90, 40, g_valveOn ? "ON" : "off");
  if (g_auto) u8g2.drawStr(95, 54, "AUTO");
  u8g2.sendBuffer();
}

void handleSerial() {
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n');
  c.trim(); c.toLowerCase();
  if      (c == "auto")  { g_auto = true;  Serial.println(F("auto on")); }
  else if (c == "stop")  { g_auto = false; valveWrite(g_valve, false); Serial.println(F("stop")); }
  else if (c == "alloff"){ g_auto = false; valveWrite(-1, false); Serial.println(F("all off")); }
  else if (c.startsWith("v") && (c.endsWith("on") || c.endsWith("off"))) {
    bool on = c.endsWith("on");
    int idx = c.startsWith("vr") ? 5 : (c.charAt(1) - '1');
    if (idx >= 0 && idx < 6) { g_valve = idx; valveWrite(idx, on); }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  for (int i = 0; i < 6; i++) pinMode(VALVE_PINS[i], OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PRESS, INPUT_PULLUP);
  pinMode(BTN_TIME,  INPUT_PULLUP);
  valveWrite(-1, false);
  u8g2.begin();
  Serial.println(F("\nVerveFlo Valve Test — btn37=valve  btn48=dwell"));
  draw();
}

void loop() {
  handleSerial();

  // button: cycle valve (active LOW, debounced on falling edge)
  bool p = digitalRead(BTN_PRESS);
  if (p == LOW && g_pPress == HIGH) {
    g_valve = (g_valve + 1) % 6;
    valveWrite(g_valve, false);
    Serial.printf("valve -> %s (GPIO%d)\n", VALVE_NAME[g_valve], VALVE_PINS[g_valve]);
    draw(); delay(30);
  }
  g_pPress = p;

  // button: cycle dwell
  bool t = digitalRead(BTN_TIME);
  if (t == LOW && g_pTime == HIGH) {
    g_dwellIx = (g_dwellIx + 1) % 5;
    Serial.printf("dwell -> %d ms\n", DWELLS[g_dwellIx]);
    draw(); delay(30);
  }
  g_pTime = t;

  // auto-cycle the selected valve on/off at the dwell interval
  if (g_auto && millis() - g_lastStep >= DWELLS[g_dwellIx]) {
    g_lastStep = millis();
    valveWrite(g_valve, !g_valveOn);
    draw();
  }
}
