/*
 * VerveFlo — Pressure Sensor Calibration
 * ESP32-S3 Dev Module · HX710B raw bit-bang · NO libraries
 *
 * Purpose: find your atmosphere zero (CALIB_ZERO) and check the
 * raw-to-mmHg conversion before locking constants into main firmware.
 *
 * BOARD: ESP32S3 Dev Module · USB CDC On Boot: Enabled
 * SERIAL: 115200 baud, Newline
 *
 * COMMANDS:
 *   raw       single raw read
 *   zero      average 50 reads at atmosphere -> sets CALIB_ZERO live
 *   stream    live raw + adj + mmHg (any char stops)
 *   m <val>   set CALIB_M live (e.g. m 0.00000625)
 *   show      print current constants
 *
 * WORKFLOW:
 *   1. Power on, valves/pump OFF, sensor open to atmosphere
 *   2. type 'zero'  -> records atmosphere baseline
 *   3. apply known pressure, type 'stream', read mmHg
 *   4. adjust 'm' until mmHg matches a reference gauge
 *   5. copy final CALIB_M and CALIB_ZERO into verveflo.ino
 */

#define HX_DATA_PIN  34   // Dout
#define HX_CLK_PIN   33   // SCK

float CALIB_M    = 0.00000625f;
float CALIB_B    = 0.0f;
long  CALIB_ZERO = 8388000L;
float g_lastP    = 0;

long readHX710B() {
  unsigned long t0 = millis();
  while (digitalRead(HX_DATA_PIN)) {
    if (millis() - t0 > 200) return -1;
  }
  long value = 0;
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(HX_CLK_PIN, HIGH); delayMicroseconds(1);
    value = (value << 1) | digitalRead(HX_DATA_PIN);
    digitalWrite(HX_CLK_PIN, LOW);  delayMicroseconds(1);
  }
  digitalWrite(HX_CLK_PIN, HIGH); delayMicroseconds(1);
  digitalWrite(HX_CLK_PIN, LOW);
  if (value & 0x800000) value |= 0xFF000000;
  return value;
}

float rawToMmHg(long raw) {
  if (raw > 16000000L || raw < 0) return g_lastP;
  float mmhg = CALIB_M * (float)(raw - CALIB_ZERO) + CALIB_B;
  mmhg = constrain(mmhg, 0.0f, 250.0f);
  g_lastP = mmhg;
  return mmhg;
}

void doZero() {
  long sum = 0; int n = 0;
  for (int i = 0; i < 50; i++) {
    long r = readHX710B();
    if (r > 0 && r < 16000000L) { sum += r; n++; }
    delay(20);
  }
  if (n > 0) {
    CALIB_ZERO = sum / n;
    Serial.printf("CALIB_ZERO set to %ld (avg of %d)\n", CALIB_ZERO, n);
  } else Serial.println(F("zero failed — check wiring"));
}

void doStream() {
  Serial.println(F("stream (any char stops)..."));
  while (!Serial.available()) {
    long r = readHX710B();
    Serial.printf("raw=%ld  adj=%ld  %.2f mmHg\n", r, r - CALIB_ZERO, rawToMmHg(r));
    delay(200);
  }
  while (Serial.available()) Serial.read();
}

void showConst() {
  Serial.printf("CALIB_M=%.10f  CALIB_B=%.4f  CALIB_ZERO=%ld\n", CALIB_M, CALIB_B, CALIB_ZERO);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(HX_CLK_PIN, OUTPUT);
  pinMode(HX_DATA_PIN, INPUT);
  Serial.println(F("\nVerveFlo Pressure Calibration"));
  Serial.println(F("type: raw | zero | stream | m <val> | show"));
  showConst();
}

void loop() {
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n');
  c.trim(); c.toLowerCase();
  if      (c == "raw")    { long r = readHX710B(); Serial.printf("raw=%ld  %.2f mmHg\n", r, rawToMmHg(r)); }
  else if (c == "zero")   doZero();
  else if (c == "stream") doStream();
  else if (c == "show")   showConst();
  else if (c.startsWith("m ")) { CALIB_M = c.substring(2).toFloat(); showConst(); }
  else Serial.println(F("?"));
}
