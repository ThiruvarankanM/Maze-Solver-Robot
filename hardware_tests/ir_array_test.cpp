/*
  IR Sensor Array Tester (Arduino Mega 2560) - 8 channels
  -------------------------------------------------------
  WIRING
    Sensor VCC -> 5V
    Sensor GND -> GND
    Sensor emitter supply -> ICSP header pin 2 (5V)
      ICSP pin layout (6-pin header near the ATmega2560):   1 MISO  2 5V
                                                            3 SCK   4 MOSI
                                                            5 RESET 6 GND
      (Any other 5V pin on the Mega also works, e.g. the power header or
       the 5V pins at the top of the double-row header next to D22/D23.)
    (Emitters are always ON, so the ON/OFF emitter check is disabled.
     Use a phone camera to confirm every IR LED glows.)

    ANALOG mode : channels D1..D8 -> A0..A7   (same as IR_PINS in pins.h)
    DIGITAL mode: channels D1..D8 -> D42..D49
                  (D2-D9 are not used on the Mega because D2/D3 are the left
                   encoder and D5-D8 are the motor PWM pins in pins.h)

  (Optional: if you later wire the emitter ENABLE/LEDON pin to D10 instead,
   set USE_EMITTER_PIN to 1 to get the automatic per-channel emitter check.)

  WHAT IT TESTS
    1. Reflectance: sweep over black/white -> each channel should swing.
    2. Emitter check (analog mode): emitters are switched OFF then ON for each
       reading. A working emitter makes a big ON/OFF difference over white.
       "EMIT?" = no difference -> emitter LED for that channel likely dead.

  Serial Monitor: 115200 baud. Type 'r' to reset min/max.
*/
#include <Arduino.h>

#define USE_ANALOG      1   // 1 = analog outputs, 0 = digital outputs
#define USE_EMITTER_PIN 0   // 0 = emitters powered from 5V (always ON)
#define EMITTER_PIN     10  // only used when USE_EMITTER_PIN = 1 (D10 is free in pins.h)

#if USE_ANALOG
  //                         D1  D2  D3  D4  D5  D6  D7  D8   (left -> right)
  const int sensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
  const int SWING_OK = 200;   // min reflectance change (of 1023)
  const int EMIT_OK  = 100;   // min ON-vs-OFF difference (of 1023)
#else
  //                         D1  D2  D3  D4  D5  D6  D7  D8
  const int sensorPins[] = {42, 43, 44, 45, 46, 47, 48, 49};
  const int SWING_OK = 1;
#endif
const int N = sizeof(sensorPins) / sizeof(sensorPins[0]);

int minVal[8], maxVal[8], maxEmitDiff[8];

void resetMinMax() {
  for (int i = 0; i < N; i++) { minVal[i] = 1023; maxVal[i] = 0; maxEmitDiff[i] = 0; }
}

void emitters(bool on) {
#if USE_EMITTER_PIN
  digitalWrite(EMITTER_PIN, on ? HIGH : LOW);
  delayMicroseconds(300);   // let LEDs / phototransistors settle
#else
  (void)on;
#endif
}

int readSensor(int i) {
#if USE_ANALOG
  return analogRead(sensorPins[i]);
#else
  return digitalRead(sensorPins[i]);
#endif
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < N; i++) pinMode(sensorPins[i], INPUT);
#if USE_EMITTER_PIN
  pinMode(EMITTER_PIN, OUTPUT);
  digitalWrite(EMITTER_PIN, HIGH);
#endif
  resetMinMax();
  Serial.println(F("\n=== IR Sensor Array Test (Mega 2560, 8 ch) ==="));
  Serial.println(F("Sweep the sensor over black and white. Type 'r' to reset.\n"));
}

void loop() {
  if (Serial.available() && Serial.read() == 'r') {
    resetMinMax();
    Serial.println(F("\n-- min/max reset --\n"));
  }

  int v[8];
#if USE_ANALOG && USE_EMITTER_PIN
  int off[8];
  emitters(false);
  for (int i = 0; i < N; i++) off[i] = readSensor(i);
#endif

  emitters(true);
  for (int i = 0; i < N; i++) {
    v[i] = readSensor(i);
    if (v[i] < minVal[i]) minVal[i] = v[i];
    if (v[i] > maxVal[i]) maxVal[i] = v[i];
#if USE_ANALOG && USE_EMITTER_PIN
    int d = abs(v[i] - off[i]);
    if (d > maxEmitDiff[i]) maxEmitDiff[i] = d;
#endif
  }

  // Live values (emitters ON)
  Serial.print(F("Live: "));
  for (int i = 0; i < N; i++) {
#if USE_ANALOG
    char buf[6]; sprintf(buf, "%4d ", v[i]); Serial.print(buf);
#else
    Serial.print(v[i] ? F("# ") : F(". "));   // # = HIGH, . = LOW
#endif
  }

  // Status per channel
  Serial.print(F(" | "));
  for (int i = 0; i < N; i++) {
    Serial.print(F("S")); Serial.print(i + 1);
    bool swingOK = (maxVal[i] - minVal[i]) >= SWING_OK;
#if USE_ANALOG && USE_EMITTER_PIN
    bool emitOK = maxEmitDiff[i] >= EMIT_OK;
    if (swingOK && emitOK) Serial.print(F(":OK "));
    else if (!emitOK)      Serial.print(F(":EMIT? "));
    else                   Serial.print(F(":DEAD? "));
#else
    Serial.print(swingOK ? F(":OK ") : F(":DEAD? "));
#endif
  }
  Serial.println();

  // Summary every ~3 s
  static unsigned long lastSum = 0;
  if (millis() - lastSum > 3000) {
    lastSum = millis();
#if USE_ANALOG && USE_EMITTER_PIN
    Serial.println(F("\n  Ch   Min   Max  Swing  On-Off"));
#else
    Serial.println(F("\n  Ch   Min   Max  Swing"));
#endif
    for (int i = 0; i < N; i++) {
      char buf[40];
#if USE_ANALOG && USE_EMITTER_PIN
      sprintf(buf, "  S%d  %4d  %4d  %4d   %4d", i + 1, minVal[i], maxVal[i],
              maxVal[i] - minVal[i], maxEmitDiff[i]);
#else
      sprintf(buf, "  S%d  %4d  %4d  %4d", i + 1, minVal[i], maxVal[i], maxVal[i] - minVal[i]);
#endif
      Serial.println(buf);
    }
    Serial.println();
  }

  delay(150);
}
