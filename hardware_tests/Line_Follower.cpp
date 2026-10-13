// ===================================================================
//  BLACK LINE FOLLOWER  —  Arduino Mega 2560
//  8-ch analog IR array (analogRead), 2x HW-039 (BTS7960), PD control
// -------------------------------------------------------------------
//  Sketch folder:  LineFollower/LineFollower.ino  +  LineFollower/pins.h
//
//  WIRING
//    IR array : OUT1..OUT8 -> A0..A7 (IR_PINS in pins.h, LEFT -> RIGHT)
//               VCC -> 5V, GND -> GND, emitter supply -> another 5V pin
//               (Mega has spare 5V pins on the power header and at the
//                top of the double-row header next to pins 22/23)
//    Motors   : as in pins.h (RPWM = forward, LPWM = reverse, EN bridged)
//    Button   : RUN_BUTTON (pin 40) -> GND, optional
//
//  IR LOGIC  : raw < 650 = WHITE,  raw > 850 = BLACK
//              650..850 = keeps previous state (hysteresis), and is used
//              as a partial value for a smooth line position.
//
//  SERIAL 115200:
//    g = GO                 d = dry run (motors off, robot moved by hand)
//    any key = STOP         m = motor test (lift the robot first!)
//    t = telemetry on/off   ? = show settings
//    P0.03  D0.3  I0  V80  X150  -> set Kp, Kd, Ki, base speed, max speed
//  BUTTON: press = start / stop
// ===================================================================
#include <Arduino.h>
#include "pins.h"

// ---------------- IR ARRAY ----------------
const uint8_t NUM_SENSORS   = 8;
const int     WHITE_MAX     = 650;    // raw below this  -> white
const int     BLACK_MIN     = 850;    // raw above this  -> black
const bool    REVERSE_ORDER = false;  // set true if line under LEFT sensor shows pos ~7000
const uint8_t WIDE_COUNT    = 6;      // >= this many black = junction / wide patch

// ---------------- MOTORS ----------------
const bool L_REVERSED = false;        // flip if 'm' test shows a wheel turning the wrong way
const bool R_REVERSED = true;

// ---------------- BUTTON ----------------
// pins.h START_BUTTON (31) clashes with US_FRONT_ECHO, so a free pin is used here.
const uint8_t RUN_BUTTON = 40;

// ---------------- TUNING ----------------
float Kp = 0.025;        // error is -3500..+3500, correction is PWM
float Kd = 0.25;
float Ki = 0.0;
int   baseSpeed   = 70;  // straight-line PWM (0..255)
int   maxSpeed    = 140; // per-wheel cap
int   searchSpeed = 70;  // spin speed while searching for a lost line

const uint16_t CONTROL_PERIOD_US = 4000;  // 250 Hz control loop
const uint8_t  SLEW_PER_STEP     = 6;     // max PWM change per step (soft start)
const bool     SEARCH_WHEN_LOST  = true;  // true: spin toward last seen side; false: stop
const uint16_t LOST_TIMEOUT_MS   = 1000;  // stop if line not found within this time
const uint16_t LOST_STOP_MS      = 40;    // (stop mode) ignore glitches shorter than this
const uint16_t ALL_BLACK_STOP_MS = 300;   // all 8 black this long = lifted -> stop

// ---------------- STATE ----------------
int  raw[NUM_SENSORS];
int  norm[NUM_SENSORS];    // 0 = white .. 1000 = black
bool black[NUM_SENSORS];   // thresholded with hysteresis
int  lastPos = 3500;       // 0 = far left, 3500 = centre, 7000 = far right

bool  running = false, dryRun = false, telemetry = false;
int   lastError = 0;
float integral  = 0;
int   curL = 0, curR = 0;

// =================== MOTORS ===================
void setMotor(uint8_t fwdPin, uint8_t revPin, int speed, bool rev) {
  if (rev) speed = -speed;
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { analogWrite(revPin, 0); analogWrite(fwdPin, speed); }
  else            { analogWrite(fwdPin, 0); analogWrite(revPin, -speed); }
}

void drive(int left, int right) {
  curL = left; curR = right;
  setMotor(MOTOR_LEFT_RPWM,  MOTOR_LEFT_LPWM,  left,  L_REVERSED);
  setMotor(MOTOR_RIGHT_RPWM, MOTOR_RIGHT_LPWM, right, R_REVERSED);
}

void driveSmooth(int left, int right) {
  if (dryRun) { drive(0, 0); return; }
  left  = constrain(left,  curL - SLEW_PER_STEP, curL + SLEW_PER_STEP);
  right = constrain(right, curR - SLEW_PER_STEP, curR + SLEW_PER_STEP);
  drive(left, right);
}

void motorTest() {
  Serial.println(F(">> Motor test - LIFT THE ROBOT. Each wheel should turn as named."));
  const int seq[4][2] = { {120, 0}, {-120, 0}, {0, 120}, {0, -120} };
  const char *const names[4] = { "Left  FORWARD", "Left  BACKWARD", "Right FORWARD", "Right BACKWARD" };
  for (uint8_t k = 0; k < 4; k++) {
    Serial.println(names[k]);
    drive(seq[k][0], seq[k][1]); delay(800);
    drive(0, 0);                 delay(400);
  }
  Serial.println(F(">> Wrong direction? Flip L_REVERSED / R_REVERSED."));
}

// =================== IR ARRAY ===================
void readSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint8_t ch = REVERSE_ORDER ? (NUM_SENSORS - 1 - i) : i;
    int v = analogRead(IR_PINS[ch]);
    raw[i] = v;

    if (v >= BLACK_MIN)      black[i] = true;   // hysteresis: in-between keeps old state
    else if (v <= WHITE_MAX) black[i] = false;

    long n = (long)(v - WHITE_MAX) * 1000L / (BLACK_MIN - WHITE_MAX);
    norm[i] = (int)constrain(n, 0L, 1000L);
  }
}

// Weighted average of the 8 sensors -> 0..7000
int readPosition(bool &lost, uint8_t &active) {
  readSensors();
  long weighted = 0, sum = 0;
  active = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (black[i]) active++;
    weighted += (long)norm[i] * (i * 1000L);
    sum      += norm[i];
  }
  lost = (active == 0);
  if (!lost && sum > 0) lastPos = weighted / sum;
  return lastPos;
}

void printBars() {
  Serial.print('|');
  for (uint8_t i = 0; i < NUM_SENSORS; i++) Serial.print(black[i] ? '#' : '.');
  Serial.print(F("| "));
}

// =================== RUN CONTROL ===================
void startRun(bool dry) {
  dryRun = dry;
  integral = 0; lastError = 0; lastPos = 3500;
  drive(0, 0);
  running = true;
  Serial.println(dry ? F(">> DRY RUN (motors off) - move the robot by hand")
                     : F(">> GO"));
  delay(300);
}

void stopRun(const __FlashStringHelper *why) {
  running = false;
  drive(0, 0);
  Serial.print(F(">> STOP: ")); Serial.println(why);
}

void followStep() {
  static unsigned long lostSince = 0, darkSince = 0;
  static bool wasLost = false;

  bool lost; uint8_t active;
  int pos   = readPosition(lost, active);
  int error = pos - 3500;          // + = line is to the right

  // ---- all sensors black for a while = robot lifted / off the track ----
  if (active == NUM_SENSORS) {
    if (darkSince == 0) darkSince = millis();
    if (!dryRun && millis() - darkSince > ALL_BLACK_STOP_MS) {
      darkSince = 0; stopRun(F("all sensors black (lifted?)")); return;
    }
  } else darkSince = 0;

  // ---- line lost ----
  if (lost) {
    if (lostSince == 0) lostSince = millis();
    wasLost = true;
    unsigned long limit = SEARCH_WHEN_LOST ? LOST_TIMEOUT_MS : LOST_STOP_MS;
    if (millis() - lostSince > limit) {
      lostSince = 0;
      if (dryRun) { Serial.println(F("LOST -> would STOP")); return; }
      stopRun(F("line lost")); return;
    }
    if (SEARCH_WHEN_LOST) {
      integral = 0;
      if (lastPos < 3500) driveSmooth(-searchSpeed,  searchSpeed);   // line went left
      else                driveSmooth( searchSpeed, -searchSpeed);   // line went right
    }
    return;
  }
  lostSince = 0;
  if (wasLost) { lastError = error; wasLost = false; }   // no derivative kick on re-acquire

  // ---- wide patch / junction: go straight through ----
  if (active >= WIDE_COUNT) { driveSmooth(baseSpeed, baseSpeed); return; }

  // ---- PID ----
  integral = constrain(integral + error, -50000.0f, 50000.0f);
  int derivative = error - lastError;
  lastError = error;
  float corr = Kp * error + Kd * derivative + Ki * integral;

  // slow down in curves (down to 50% at the edge of the array)
  int speed = baseSpeed - (int)((long)abs(error) * baseSpeed / 7000L);

  int left  = constrain(speed + (int)corr, -maxSpeed, maxSpeed);
  int right = constrain(speed - (int)corr, -maxSpeed, maxSpeed);
  driveSmooth(left, right);

  if (telemetry || dryRun) {
    static unsigned long lastT = 0;
    if (millis() - lastT >= 100) {
      lastT = millis();
      printBars();
      Serial.print(F("err="));  Serial.print(error);
      Serial.print(F("  L="));  Serial.print(left);
      Serial.print(F("  R="));  Serial.println(right);
    }
  }
}

// =================== UI ===================
void printSettings() {
  Serial.print(F("Kp="));     Serial.print(Kp, 4);
  Serial.print(F("  Kd="));   Serial.print(Kd, 4);
  Serial.print(F("  Ki="));   Serial.print(Ki, 5);
  Serial.print(F("  base=")); Serial.print(baseSpeed);
  Serial.print(F("  max="));  Serial.print(maxSpeed);
  Serial.print(F("  white<")); Serial.print(WHITE_MAX);
  Serial.print(F("  black>")); Serial.println(BLACK_MIN);
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r') return;
  if (running && c != 't') { stopRun(F("serial")); return; }

  switch (c) {
    case 'g': startRun(false); break;
    case 'd': startRun(true);  break;
    case 'm': motorTest();     break;
    case 't': telemetry = !telemetry;
              Serial.print(F(">> telemetry = ")); Serial.println(telemetry); break;
    case 'P': Kp = Serial.parseFloat(); printSettings(); break;
    case 'D': Kd = Serial.parseFloat(); printSettings(); break;
    case 'I': Ki = Serial.parseFloat(); printSettings(); break;
    case 'V': baseSpeed = constrain(Serial.parseInt(), 0, 255); printSettings(); break;
    case 'X': maxSpeed  = constrain(Serial.parseInt(), 0, 255); printSettings(); break;
    case '?': printSettings(); break;
  }
}

void handleButton() {
  static bool lastState = HIGH;
  static unsigned long lastChange = 0;
  bool s = digitalRead(RUN_BUTTON);
  if (s != lastState && millis() - lastChange > 50) {   // debounce
    lastChange = millis();
    lastState = s;
    if (s == LOW) {
      if (running) stopRun(F("button"));
      else         startRun(false);
    }
  }
}

// =================== SETUP / LOOP ===================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(RUN_BUTTON, INPUT_PULLUP);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) pinMode(IR_PINS[i], INPUT);

  pinMode(MOTOR_LEFT_RPWM,  OUTPUT); pinMode(MOTOR_LEFT_LPWM,  OUTPUT); pinMode(MOTOR_LEFT_EN,  OUTPUT);
  pinMode(MOTOR_RIGHT_RPWM, OUTPUT); pinMode(MOTOR_RIGHT_LPWM, OUTPUT); pinMode(MOTOR_RIGHT_EN, OUTPUT);
  drive(0, 0);
  digitalWrite(MOTOR_LEFT_EN, HIGH);
  digitalWrite(MOTOR_RIGHT_EN, HIGH);

  Serial.println(F("\n=== Line Follower (Mega 2560) ==="));
  Serial.println(F("g=go d=dryrun anykey=stop m=motortest t=telemetry P D I V X ?"));
  printSettings();
}

void loop() {
  handleSerial();
  handleButton();

  if (running) {
    static unsigned long lastCtrl = 0;
    unsigned long now = micros();
    if (now - lastCtrl >= CONTROL_PERIOD_US) { lastCtrl = now; followStep(); }
    return;
  }

  // ---- idle: live sensor view at 10 Hz ----
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  bool lost; uint8_t active;
  int pos = readPosition(lost, active);
  printBars();
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    char buf[6]; sprintf(buf, "%4d ", raw[i]); Serial.print(buf);
  }
  Serial.print(F(" pos="));     Serial.print(pos);
  Serial.print(F("  active=")); Serial.print(active);
  if (lost) Serial.print(F("  LINE LOST"));
  Serial.println();
}
