// ===== QYF-750 (QTR-8RC type) IR ARRAY TEST + CALIBRATION — EE4360 =====
// Sensors D1..D8 -> A8..A15 (PORTK bit0..bit7), read as RC timing
#include <EEPROM.h>

const uint8_t  NUM_SENSORS    = 8;
const int8_t   IR_EMITTER_PIN = 36;    // board "IR" pin; -1 if not connected
const uint8_t  CAL_BUTTON     = 40;    // button to GND
const uint16_t TIMEOUT_US     = 2500;  // max discharge time (= "fully dark")
const uint16_t CAL_TIME_MS    = 6000;
const uint16_t LINE_THRESH    = 300;   // normalized > this = on line
const uint16_t MIN_RANGE      = 200;   // us; warn if contrast below this
const bool     REVERSE_ORDER  = false; // true if D1 is on the RIGHT side of your robot

struct CalData {
  uint16_t magic;
  uint16_t minV[NUM_SENSORS];
  uint16_t maxV[NUM_SENSORS];
  bool invert;
};
const uint16_t CAL_MAGIC = 0xC750;     // new magic so old analog cal isn't loaded
CalData cal;

uint16_t raw[NUM_SENSORS], norm[NUM_SENSORS];
bool calibrated = false;
int lastPosition = 3500;
enum Mode { MODE_RAW, MODE_NORM, MODE_POS } mode = MODE_RAW;

// ---------- RC reading (all 8 in parallel on PORTK) ----------
void readRaw() {
  uint16_t t8[NUM_SENSORS];
  for (uint8_t i = 0; i < NUM_SENSORS; i++) t8[i] = TIMEOUT_US;

  if (IR_EMITTER_PIN >= 0) { digitalWrite(IR_EMITTER_PIN, HIGH); delayMicroseconds(200); }

  DDRK  = 0xFF;  PORTK = 0xFF;          // drive all HIGH -> charge capacitors
  delayMicroseconds(10);
  DDRK  = 0x00;  PORTK = 0x00;          // switch to input, no pull-ups

  uint32_t start = micros();
  uint8_t pending = 0xFF;
  while (pending) {
    uint32_t t = micros() - start;
    if (t >= TIMEOUT_US) break;
    uint8_t fell = pending & ~PINK;      // bits that just went LOW
    if (fell) {
      for (uint8_t i = 0; i < NUM_SENSORS; i++)
        if (fell & (1 << i)) t8[i] = t;
      pending &= ~fell;
    }
  }

  if (IR_EMITTER_PIN >= 0) digitalWrite(IR_EMITTER_PIN, LOW);   // saves power between reads

  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    raw[i] = REVERSE_ORDER ? t8[NUM_SENSORS - 1 - i] : t8[i];
}

// 0 = background, 1000 = line
void readNormalized() {
  readRaw();
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    long range = (long)cal.maxV[i] - cal.minV[i];
    long v = 0;
    if (range > 0) v = ((long)raw[i] - cal.minV[i]) * 1000L / range;
    v = constrain(v, 0, 1000);
    if (cal.invert) v = 1000 - v;
    norm[i] = v;
  }
}

// 0..7000, 3500 = centred
int readPosition(bool &lost, uint8_t &activeCount) {
  readNormalized();
  long weighted = 0, sum = 0;
  activeCount = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (norm[i] > LINE_THRESH) activeCount++;
    if (norm[i] > 50) { weighted += (long)norm[i] * (i * 1000L); sum += norm[i]; }
  }
  lost = (activeCount == 0);
  if (lost) return (lastPosition < 3500) ? 0 : 7000;
  lastPosition = weighted / sum;
  return lastPosition;
}

// ---------- calibration ----------
void calibrate() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { cal.minV[i] = TIMEOUT_US; cal.maxV[i] = 0; }
  Serial.println(F("\n>> CALIBRATING: sweep the array slowly across the line, back and forth..."));
  unsigned long start = millis(), lastTick = 0;
  while (millis() - start < CAL_TIME_MS) {
    readRaw();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < cal.minV[i]) cal.minV[i] = raw[i];
      if (raw[i] > cal.maxV[i]) cal.maxV[i] = raw[i];
    }
    if (millis() - lastTick >= 1000) {
      lastTick = millis();
      Serial.print((CAL_TIME_MS - (millis() - start)) / 1000 + 1); Serial.println(F("s..."));
    }
  }
  calibrated = true;
  Serial.println(F(">> Done.\nS#\tmin(us)\tmax(us)\trange"));
  bool ok = true;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t r = cal.maxV[i] - cal.minV[i];
    Serial.print(i + 1); Serial.print('\t'); Serial.print(cal.minV[i]); Serial.print('\t');
    Serial.print(cal.maxV[i]); Serial.print('\t'); Serial.print(r);
    if (r < MIN_RANGE) { Serial.print(F("  <-- LOW CONTRAST")); ok = false; }
    Serial.println();
  }
  if (!ok) Serial.println(F("!! Some sensors never saw both surfaces. Sweep wider or lower the array (~3 mm)."));
  Serial.println(F(">> Centre the array on the line, press 'n'. On-line sensors should read ~1000."));
  Serial.println(F("   If ~0, press 'i'. Then 's' to save."));
}

void saveCal() { cal.magic = CAL_MAGIC; EEPROM.put(0, cal); Serial.println(F(">> Saved to EEPROM")); }
bool loadCal() {
  CalData tmp; EEPROM.get(0, tmp);
  if (tmp.magic != CAL_MAGIC) return false;
  cal = tmp; calibrated = true; return true;
}

void printBars() {
  Serial.print('|');
  for (uint8_t i = 0; i < NUM_SENSORS; i++) Serial.print(norm[i] > LINE_THRESH ? '#' : '.');
  Serial.print("| ");
}

void setup() {
  Serial.begin(115200);
  pinMode(CAL_BUTTON, INPUT_PULLUP);
  if (IR_EMITTER_PIN >= 0) { pinMode(IR_EMITTER_PIN, OUTPUT); digitalWrite(IR_EMITTER_PIN, LOW); }
  cal.invert = false;
  if (loadCal()) { Serial.println(F("Calibration loaded.")); mode = MODE_POS; }
  else           { Serial.println(F("No calibration saved. Showing RAW (us). Press 'c' or the button.")); }
  Serial.println(F("Cmds: c=cal r=raw n=norm p=pos i=invert s=save l=load x=clear"));
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 'c': calibrate(); break;
      case 'r': mode = MODE_RAW; break;
      case 'n': if (calibrated) mode = MODE_NORM; else Serial.println(F("Calibrate first")); break;
      case 'p': if (calibrated) mode = MODE_POS;  else Serial.println(F("Calibrate first")); break;
      case 'i': cal.invert = !cal.invert; Serial.print(F(">> invert = ")); Serial.println(cal.invert); break;
      case 's': saveCal(); break;
      case 'l': Serial.println(loadCal() ? F(">> Loaded") : F(">> Nothing saved")); break;
      case 'x': EEPROM.put(0, (uint16_t)0); calibrated = false; mode = MODE_RAW; Serial.println(F(">> Cleared")); break;
    }
  }

  if (digitalRead(CAL_BUTTON) == LOW) {
    delay(30);
    if (digitalRead(CAL_BUTTON) == LOW) {
      while (digitalRead(CAL_BUTTON) == LOW);
      calibrate(); saveCal(); mode = MODE_POS;
    }
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  if (mode == MODE_RAW) {
    readRaw();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(raw[i]); Serial.print('\t'); }
    Serial.println();
  } else if (mode == MODE_NORM) {
    readNormalized(); printBars();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(norm[i]); Serial.print('\t'); }
    Serial.println();
  } else {
    bool lost; uint8_t active;
    int pos = readPosition(lost, active);
    printBars();
    Serial.print(F("pos=")); Serial.print(pos);
    Serial.print(F("  error=")); Serial.print(pos - 3500);
    Serial.print(F("  active=")); Serial.print(active);
    if (lost) Serial.print(F("  LINE LOST"));
    if (active >= 6) Serial.print(F("  WIDE PATCH (tile/marker?)"));
    Serial.println();
  }
}
