/*
 * VerveFlo — Hardware Test
 * ESP32-S3 Dev Module · Arduino IDE · NO external libraries
 *
 * Upload this BEFORE the main firmware to verify wiring.
 * Raw bit-bang HX710B read (no Q2HX711 needed).
 *
 * BOARD SETTINGS:
 *   Board           : ESP32S3 Dev Module
 *   USB CDC On Boot : Enabled   <- MUST, else no serial
 *   Upload Speed    : 921600
 *
 * SERIAL: 115200 baud, Newline line ending
 *
 * COMMANDS:
 *   selftest   full auto test (LED, battery, pressure, pump 2s, all valves 1s)
 *   led        blink LED 5x
 *   pump on    pump motor ON          pump off   pump OFF
 *   v1..v5 on/off   fill valves
 *   vr on/off       release valve (raw HIGH/LOW — see note below)
 *   alloff     everything OFF immediately
 *   pressure   single raw read + mmHg
 *   pstream    live pressure stream + bar graph (any key stops)
 *   battery    battery raw + voltage + percent
 *   pins       print all output pin states
 *   buttons    print all button states (hold a button while typing)
 *
 * NOTE on release valve (GPIO42): it is normally-open in the real system.
 * This raw test just drives the pin HIGH/LOW so you can confirm wiring +
 * the driver. Pneumatic open/close meaning is handled in the main firmware.
 */

// ── PINS (verified hardware sheet) ───────────────
// Index: 0=Fill1, 1=Fill2, 2=Fill3, 3=Fill4, 4=Fill5, 5=Release
static const uint8_t VALVE_PINS[6] = { 26, 47, 39, 40, 41, 42 };
#define RELEASE_VALVE_IDX  5

#define PUMP_PIN     21
#define LED_PIN      3
#define HX_DATA_PIN  34   // Dout
#define HX_CLK_PIN   33   // SCK
#define BATT_ADC     15

#define BTN_START    35
#define BTN_MODE     36
#define BTN_PRESS    37
#define BTN_TIME     48

// ── Calibration (v3.1) ───────────────────────────
const float CALIB_M    = 0.00000625f;   // positive
const float CALIB_B    = 0.0f;
long        CALIB_ZERO = 8388000L;       // atmosphere ref

float    g_battV   = 0;
int      g_battPct = 0;
float    g_lastP   = 0;

// ── HX710B raw bit-bang read ─────────────────────
long readHX710B() {
  // wait until data line goes low (data ready)
  unsigned long t0 = millis();
  while (digitalRead(HX_DATA_PIN)) {
    if (millis() - t0 > 200) return -1;   // timeout
  }
  long value = 0;
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(HX_CLK_PIN, HIGH);
    delayMicroseconds(1);
    value = (value << 1) | digitalRead(HX_DATA_PIN);
    digitalWrite(HX_CLK_PIN, LOW);
    delayMicroseconds(1);
  }
  // 25th pulse — set gain/channel
  digitalWrite(HX_CLK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX_CLK_PIN, LOW);
  // sign extend 24-bit
  if (value & 0x800000) value |= 0xFF000000;
  return value;
}

float rawToMmHg(long raw) {
  if (raw > 16000000L || raw < 0) return g_lastP;   // overflow filter
  long  adj  = raw - CALIB_ZERO;
  float mmhg = CALIB_M * (float)adj + CALIB_B;
  mmhg = constrain(mmhg, 0.0f, 250.0f);
  g_lastP = mmhg;
  return mmhg;
}

void readBattery() {
  int raw = analogRead(BATT_ADC);
  // 2S pack through divider — adjust ratio to your hardware
  g_battV   = (raw / 4095.0f) * 3.3f * 2.0f;
  g_battPct = constrain(map((int)(g_battV * 100), 600, 840, 0, 100), 0, 100);
}

// ── Valve / pump helpers ─────────────────────────
void valveWrite(uint8_t idx, bool on) {
  digitalWrite(VALVE_PINS[idx], on ? HIGH : LOW);
}

void allOff() {
  for (uint8_t i = 0; i < 6; i++) digitalWrite(VALVE_PINS[i], LOW);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

// ── Self test ────────────────────────────────────
void selfTest() {
  Serial.println(F("=== SELF TEST ==="));

  Serial.print(F("LED... "));
  for (uint8_t i = 0; i < 6; i++) { digitalWrite(LED_PIN, i % 2 == 0); delay(250); }
  digitalWrite(LED_PIN, LOW);
  Serial.println(F("done"));

  readBattery();
  Serial.printf("Battery: %.2fV  %d%%\n", g_battV, g_battPct);

  long  raw  = readHX710B();
  float mmhg = rawToMmHg(raw);
  Serial.printf("Pressure: %.2f mmHg (raw=%ld)\n", mmhg, raw);

  Serial.println(F("Pump ON 2s..."));
  digitalWrite(PUMP_PIN, HIGH); delay(2000);
  digitalWrite(PUMP_PIN, LOW);

  for (uint8_t i = 0; i < 6; i++) {
    Serial.printf("Valve %d (GPIO%d) 1s...\n", i + 1, VALVE_PINS[i]);
    valveWrite(i, true);  delay(1000);
    valveWrite(i, false); delay(200);
  }
  Serial.println(F("=== SELF TEST COMPLETE ==="));
}

void printPins() {
  Serial.println(F("--- output pin states ---"));
  for (uint8_t i = 0; i < 6; i++)
    Serial.printf("Valve %d GPIO%d = %d\n", i + 1, VALVE_PINS[i], digitalRead(VALVE_PINS[i]));
  Serial.printf("Pump GPIO%d = %d\n", PUMP_PIN, digitalRead(PUMP_PIN));
  Serial.printf("LED  GPIO%d = %d\n", LED_PIN, digitalRead(LED_PIN));
}

void printButtons() {
  Serial.printf("START=%d MODE=%d PRESS=%d TIME=%d  (0=pressed, active LOW)\n",
    digitalRead(BTN_START), digitalRead(BTN_MODE),
    digitalRead(BTN_PRESS), digitalRead(BTN_TIME));
}

void pstream() {
  Serial.println(F("Live pressure (send any char to stop)..."));
  while (!Serial.available()) {
    long  raw  = readHX710B();
    float mmhg = rawToMmHg(raw);
    int   bars = constrain((int)(mmhg / 5.0f), 0, 40);
    Serial.printf("%6.1f mmHg |", mmhg);
    for (int i = 0; i < bars; i++) Serial.print('#');
    Serial.println();
    delay(200);
  }
  while (Serial.available()) Serial.read();
}

// ── Command parser ───────────────────────────────
void handleCmd(String c) {
  c.trim(); c.toLowerCase();
  if (c == "")          return;
  else if (c == "selftest") selfTest();
  else if (c == "led")  { for (uint8_t i = 0; i < 10; i++) { digitalWrite(LED_PIN, i % 2 == 0); delay(150); } digitalWrite(LED_PIN, LOW); }
  else if (c == "pump on")  { digitalWrite(PUMP_PIN, HIGH); Serial.println(F("pump ON")); }
  else if (c == "pump off") { digitalWrite(PUMP_PIN, LOW);  Serial.println(F("pump OFF")); }
  else if (c == "alloff")   { allOff(); Serial.println(F("ALL OFF")); }
  else if (c == "pressure") { long r = readHX710B(); Serial.printf("raw=%ld  %.2f mmHg\n", r, rawToMmHg(r)); }
  else if (c == "pstream")  pstream();
  else if (c == "battery")  { readBattery(); Serial.printf("%.2fV  %d%%\n", g_battV, g_battPct); }
  else if (c == "pins")     printPins();
  else if (c == "buttons")  printButtons();
  else if (c.startsWith("v") && (c.endsWith("on") || c.endsWith("off"))) {
    bool on = c.endsWith("on");
    int idx = -1;
    if (c.startsWith("vr")) idx = RELEASE_VALVE_IDX;
    else                    idx = c.charAt(1) - '1';   // v1..v5 -> 0..4
    if (idx >= 0 && idx < 6) { valveWrite(idx, on); Serial.printf("valve idx%d GPIO%d %s\n", idx, VALVE_PINS[idx], on ? "ON" : "OFF"); }
    else Serial.println(F("bad valve"));
  }
  else Serial.println(F("unknown cmd — see header for list"));
}

// ── Arduino ──────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);   // let USB CDC settle so we don't miss prints
  Serial.println(F("\nVerveFlo Hardware Test — type 'selftest'"));

  for (uint8_t i = 0; i < 6; i++) pinMode(VALVE_PINS[i], OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(HX_CLK_PIN, OUTPUT);
  pinMode(HX_DATA_PIN, INPUT);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_MODE,  INPUT_PULLUP);
  pinMode(BTN_PRESS, INPUT_PULLUP);
  pinMode(BTN_TIME,  INPUT_PULLUP);
  analogReadResolution(12);
  allOff();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCmd(line);
  }
}
