/*
 * Encoder test - LEFT + RIGHT JGA25-370 motors
 * Arduino Mega 2560 - EE4360 Maze Solver
 *
 * Encoder wiring:
 *   LEFT  : A (yellow) -> D2,  B (green) -> D3
 *   RIGHT : A (yellow) -> D18, B (green) -> D19
 *   Both  : VCC (blue) -> Mega 5V,  GND (black) -> Mega GND
 *
 * Motor driver wiring (only needed for the 'f' / 'b' commands):
 *   LEFT  HW-039 : RPWM -> D5, LPWM -> D6, EN -> D24 (or 5V)
 *   RIGHT HW-039 : RPWM -> D7, LPWM -> D8, EN -> D25 (or 5V)
 *   Battery (-) must be connected to Mega GND
 *
 * !! LIFT THE WHEELS OFF THE TABLE !!
 *
 * Serial Monitor: 115200 baud, line ending = "Newline"
 *   (nothing) -> just turn wheels by hand and watch the counts
 *   r   -> reset both counts to 0
 *   f   -> run both motors forward at low speed
 *   b   -> run both motors backward at low speed
 *   s   -> stop motors
 *   +/- -> change test speed by 20
 */

#include <Arduino.h>

// ---------------- Encoder pins ----------------
const uint8_t L_ENC_A = 2;
const uint8_t L_ENC_B = 3;
const uint8_t R_ENC_A = 18;
const uint8_t R_ENC_B = 19;

// Flip these if a wheel counts negative when it rolls FORWARD
const bool L_ENC_REVERSED = false;
const bool R_ENC_REVERSED = true;   // right motor is mirrored

// ---------------- Motor pins ----------------
const uint8_t L_RPWM = 5, L_LPWM = 6, L_EN = 24;
const uint8_t R_RPWM = 7, R_LPWM = 8, R_EN = 25;

const bool LEFT_REVERSED  = false;  // same flags as the motor test
const bool RIGHT_REVERSED = true;

const int MAX_PWM = 255;            // set to 128 for a 6V motor on 3S
int testSpeed = 80;

// ---------------- Encoder state ----------------
volatile long lCount = 0, rCount = 0;
volatile uint8_t lState = 0, rState = 0;

// Quadrature lookup: index = (oldAB << 2) | newAB
const int8_t QEM[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void leftISR() {
  uint8_t ab = (digitalRead(L_ENC_A) << 1) | digitalRead(L_ENC_B);
  lState = ((lState << 2) | ab) & 0x0F;
  lCount += QEM[lState];
}

void rightISR() {
  uint8_t ab = (digitalRead(R_ENC_A) << 1) | digitalRead(R_ENC_B);
  rState = ((rState << 2) | ab) & 0x0F;
  rCount += QEM[rState];
}

void readCounts(long &l, long &r) {
  noInterrupts();
  l = lCount;
  r = rCount;
  interrupts();
  if (L_ENC_REVERSED) l = -l;
  if (R_ENC_REVERSED) r = -r;
}

// ---------------- Motor helper ----------------
// speed: -MAX_PWM..+MAX_PWM (positive = forward)
void setMotor(uint8_t rpwm, uint8_t lpwm, int speed, bool reversed) {
  if (reversed) speed = -speed;
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed > 0)      { analogWrite(lpwm, 0); analogWrite(rpwm, speed); }
  else if (speed < 0) { analogWrite(rpwm, 0); analogWrite(lpwm, -speed); }
  else                { analogWrite(rpwm, 0); analogWrite(lpwm, 0); }
}

void runBoth(int speed) {
  setMotor(L_RPWM, L_LPWM, speed, LEFT_REVERSED);
  setMotor(R_RPWM, R_LPWM, speed, RIGHT_REVERSED);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  // Prime the state with the current pin levels
  lState = (digitalRead(L_ENC_A) << 1) | digitalRead(L_ENC_B);
  rState = (digitalRead(R_ENC_A) << 1) | digitalRead(R_ENC_B);

  // Interrupt on both channels -> x4 counts
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENC_B), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_B), rightISR, CHANGE);

  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT); pinMode(L_EN, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT); pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
  runBoth(0);

  Serial.println(F("=== Encoder test ==="));
  Serial.println(F("Turn each wheel FORWARD by hand: both counts should go UP."));
  Serial.println(F("Commands: r=reset  f=fwd  b=back  s=stop  +/-=speed"));
}

// ---------------- Loop ----------------
long lastL = 0, lastR = 0;
unsigned long lastPrint = 0;

void loop() {
  // --- Serial commands ---
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r':
        noInterrupts(); lCount = 0; rCount = 0; interrupts();
        lastL = lastR = 0;
        Serial.println(F("Counts reset"));
        break;
      case 'f': runBoth(testSpeed);  Serial.println(F("Forward"));  break;
      case 'b': runBoth(-testSpeed); Serial.println(F("Backward")); break;
      case 's': runBoth(0);          Serial.println(F("Stop"));     break;
      case '+':
        testSpeed = min(testSpeed + 20, MAX_PWM);
        Serial.print(F("Speed = ")); Serial.println(testSpeed);
        break;
      case '-':
        testSpeed = max(testSpeed - 20, 0);
        Serial.print(F("Speed = ")); Serial.println(testSpeed);
        break;
    }
  }

  // --- Print counts every 200 ms ---
  unsigned long now = millis();
  if (now - lastPrint >= 200) {
    long l, r;
    readCounts(l, r);
    float dt = (now - lastPrint) / 1000.0;

    Serial.print(F("L: "));      Serial.print(l);
    Serial.print(F("  ("));      Serial.print((l - lastL) / dt, 0);
    Serial.print(F(" cnt/s)   R: ")); Serial.print(r);
    Serial.print(F("  ("));      Serial.print((r - lastR) / dt, 0);
    Serial.println(F(" cnt/s)"));

    lastL = l;
    lastR = r;
    lastPrint = now;
  }
}
