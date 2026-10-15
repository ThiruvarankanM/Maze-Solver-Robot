/*
  Line Follower v2
  Arduino Mega 2560 — EE4360 Maze Solver robot
  8-ch analog IR array on A0..A7 (LEFT -> RIGHT), high reading = black

  Serial Monitor: 115200 baud, line ending "Newline"
  Commands:
    g = GO            x = STOP (any time)        d = dry run (motors OFF, move the robot by hand)
    t = telemetry on/off while running           s = live sensor view (any key ends)
    c = calibrate (robot centred ON the line)    C = clear calibration (fixed 800/900)
    m = motor direction test (wheels OFF ground) h = help
    P0.02 D0.15 V55 = set Kp, Kd, base PWM
  AUTO START: runs by itself 5 s after power-on. Send any key in those 5 s to stay in command mode.

  START: line anywhere under the array. The robot waits (motors off) until a sensor sees it.
  CORNER: only the left (or right) end sensor black = the line turns that way. It is remembered,
          so when the line is lost just after, the robot pivots to that side at once.
  LOST LINE: short gap while centred -> keep straight; otherwise pivot toward the side it was last seen.
  JUNCTION: both end sensors black -> straight through; all black too long -> stop (finish or lifted).
*/

#include <Arduino.h>
#include <EEPROM.h>

// ======================= IR ARRAY =======================
const uint8_t NUM_IR = 8;
const uint8_t IR_PINS[NUM_IR] = {A0, A1, A2, A3, A4, A5, A6, A7};
const bool IR_REVERSE_ORDER = true;  // set true if the line under the LEFT sensor shows pos ~7000
const int WHITE_MAX = 800;            // raw at or below = white (used when not calibrated)
const int BLACK_MIN = 900;            // raw at or above = black
const int ON_LINE   = 700;            // normalised 0..1000: above = black, below OFF_LINE = white
const int OFF_LINE  = 300;
const int NOISE     = 100;            // normalised value below this adds nothing to the position
const int CENTRE = 3500;              // position: 0 = far left, 7000 = far right

struct IrCal { uint32_t magic; int lo[NUM_IR]; int hi[NUM_IR]; };
const uint32_t IRCAL_MAGIC = 0x4C463202;
const int IRCAL_ADDR = 128;           // after the wall follower's EEPROM data (0..75)
const int CAL_PWM = 60;
const unsigned long CAL_SWEEP_MS = 350;
const int CAL_MIN_SWING = 150;        // a sensor must change this much between white and black

// ======================= SPEEDS / GAINS =======================
int BASE_PWM   = 55;
int MAX_PWM    = 110;
int SEARCH_PWM = 50;
float KP = 0.020;           // PWM per unit of error (error is -3500..+3500)
float KD = 0.15;            // PWM per unit of error change per loop
const float D_FILTER = 0.5; // smooths the derivative (1 = no smoothing)
const float CURVE_SLOW = 0.5;  // speed at the array edge = 50% of BASE_PWM
const int SLEW = 8;            // max PWM change per loop (soft start, no wheel jerks)
const unsigned long LOOP_MS = 5;  // 200 Hz control loop

const int CENTRE_BAND = 1500;         // line lost within this of centre = gap, not a turn
const unsigned long GAP_MS = 150;     // keep straight this long over a gap
const unsigned long CORNER_MEMORY_MS = 500;  // a corner seen this recently decides the pivot side
const unsigned long SEARCH_MS = 1500; // give up if the line is not found in this time
const unsigned long BLACK_STOP_MS = 300;
const unsigned long START_DELAY_MS = 5000;

// ======================= MOTORS (BTS7960 / HW-039) =======================
const uint8_t L_RPWM = 3, L_LPWM = 2, L_EN = 24;
const uint8_t R_RPWM = 7, R_LPWM = 8, R_EN = 25;
const bool LEFT_REVERSED  = false;
const bool RIGHT_REVERSED = true;

// ======================= STATE =======================
int raw[NUM_IR];
int val[NUM_IR];            // 0 = white .. 1000 = black
bool black[NUM_IR];
uint8_t nBlack = 0;
int linePos = CENTRE;
int lastSeenPos = CENTRE;
int calLo[NUM_IR], calHi[NUM_IR];
bool useCal = false;

bool running = false, dry = false, telemetry = false, waitLine = false;
int curL = 0, curR = 0;
int lastErr = 0;
float dFilt = 0;
unsigned long lostMs = 0, blackMs = 0, lastLoopMs = 0;
int8_t cornerSide = 0;      // -1 left, +1 right, 0 none: last end sensor that saw the line alone
unsigned long cornerMs = 0;
int8_t searchSide = 0;      // pivot side chosen when the line was lost

// ======================= MOTORS =======================
void setOne(uint8_t rp, uint8_t lp, bool rev, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (rev) pwm = -pwm;
  if (pwm >= 0) { analogWrite(rp, pwm); analogWrite(lp, 0); }
  else          { analogWrite(rp, 0);   analogWrite(lp, -pwm); }
}
void setMotors(int l, int r) {
  curL = l; curR = r;
  setOne(L_RPWM, L_LPWM, LEFT_REVERSED, l);
  setOne(R_RPWM, R_LPWM, RIGHT_REVERSED, r);
}
void stopMotors() { setMotors(0, 0); }

// Slew-limited drive; dry run keeps the motors off
void drive(int l, int r) {
  l = constrain(constrain(l, -MAX_PWM, MAX_PWM), curL - SLEW, curL + SLEW);
  r = constrain(constrain(r, -MAX_PWM, MAX_PWM), curR - SLEW, curR + SLEW);
  if (dry) { curL = l; curR = r; return; }
  setMotors(l, r);
}

// ======================= IR ARRAY =======================
int readIr(uint8_t i) { return analogRead(IR_PINS[IR_REVERSE_ORDER ? NUM_IR - 1 - i : i]); }

void readLine() {
  long weighted = 0, sum = 0;
  nBlack = 0;
  for (uint8_t i = 0; i < NUM_IR; i++) {
    raw[i] = readIr(i);
    int lo = useCal ? calLo[i] : WHITE_MAX;
    int hi = useCal ? calHi[i] : BLACK_MIN;
    val[i] = constrain((long)(raw[i] - lo) * 1000L / (hi - lo), 0L, 1000L);
    if (val[i] >= ON_LINE) black[i] = true;          // in between keeps the old state
    else if (val[i] <= OFF_LINE) black[i] = false;
    if (black[i]) nBlack++;
    if (val[i] < NOISE) continue;
    weighted += (long)val[i] * i * 1000L;
    sum += val[i];
  }
  if (nBlack > 0 && sum > 0) { linePos = weighted / sum; lastSeenPos = linePos; }
}

void printBars() {
  Serial.print('|');
  for (uint8_t i = 0; i < NUM_IR; i++) Serial.print(black[i] ? '#' : '.');
  Serial.print(F("| "));
}

// Sweep right, left, right over the line; every sensor must see white and black
void calibrate() {
  Serial.println(F("Calibrating - robot centred ON the line, wheels on the floor"));
  for (uint8_t i = 0; i < NUM_IR; i++) { calLo[i] = 1023; calHi[i] = 0; }
  const int8_t dir[3] = {1, -1, 1};
  const uint8_t len[3] = {1, 2, 1};
  for (uint8_t k = 0; k < 3; k++) {
    setMotors(dir[k] * CAL_PWM, -dir[k] * CAL_PWM);
    unsigned long t0 = millis();
    while (millis() - t0 < len[k] * CAL_SWEEP_MS) {
      for (uint8_t i = 0; i < NUM_IR; i++) {
        int v = readIr(i);
        calLo[i] = min(calLo[i], v);
        calHi[i] = max(calHi[i], v);
      }
    }
  }
  stopMotors();

  bool ok = true;
  for (uint8_t i = 0; i < NUM_IR; i++) {
    Serial.print(calLo[i]); Serial.print('-'); Serial.print(calHi[i]); Serial.print(' ');
    if (calHi[i] - calLo[i] < CAL_MIN_SWING) ok = false;
  }
  Serial.println();
  if (!ok) { Serial.println(F("FAILED: a sensor never saw both colours -> kept old settings")); return; }

  IrCal c; c.magic = IRCAL_MAGIC;
  for (uint8_t i = 0; i < NUM_IR; i++) { c.lo[i] = calLo[i]; c.hi[i] = calHi[i]; }
  EEPROM.put(IRCAL_ADDR, c);
  useCal = true;
  Serial.println(F("Calibration OK, saved"));
}

void clearCalibration() {
  IrCal c; c.magic = 0;
  EEPROM.put(IRCAL_ADDR, c);
  useCal = false;
  Serial.println(F("Calibration cleared -> fixed 800/900"));
}

// ======================= RUN CONTROL =======================
void startRun(bool dryRun) {
  dry = dryRun;
  stopMotors();
  lastErr = 0; dFilt = 0; lostMs = 0; blackMs = 0;
  lastSeenPos = CENTRE;
  cornerSide = 0; searchSide = 0;
  waitLine = true;
  running = true;
  lastLoopMs = millis();
  Serial.println(dry ? F("DRY RUN (motors off) - move the robot by hand")
                     : F("GO - waiting for the line"));
}

void endRun(const __FlashStringHelper* why) {
  if (dry) return;
  running = false;
  stopMotors();
  Serial.print(F("STOP: ")); Serial.println(why);
}

// Line missing: recent corner -> pivot that way; gap -> straight; else pivot toward the last side
void lostLine(unsigned long now) {
  if (lostMs == 0) {
    lostMs = now;
    searchSide = (cornerSide != 0 && now - cornerMs < CORNER_MEMORY_MS) ? cornerSide : 0;
    Serial.print(F("[LINE] lost, last pos ")); Serial.print(lastSeenPos);
    if (searchSide < 0) Serial.println(F(" -> corner LEFT"));
    else if (searchSide > 0) Serial.println(F(" -> corner RIGHT"));
    else Serial.println();
  }
  unsigned long t = now - lostMs;
  if (t > SEARCH_MS) { endRun(F("line lost")); return; }
  int8_t side = searchSide;
  if (side == 0) {
    if (abs(lastSeenPos - CENTRE) < CENTRE_BAND && t < GAP_MS) { drive(BASE_PWM, BASE_PWM); return; }
    side = (lastSeenPos < CENTRE) ? -1 : 1;
  }
  drive(side * SEARCH_PWM, -side * SEARCH_PWM);
}

// ======================= MAIN STEP =======================
void followStep() {
  readLine();
  unsigned long now = millis();
  int err = linePos - CENTRE;   // + = line is to the right

  if (waitLine) {
    if (nBlack == 0) { stopMotors(); return; }
    waitLine = false;
    lastErr = err;
    Serial.println(F("[LINE] found -> following"));
  }

  if (nBlack == NUM_IR) {
    if (blackMs == 0) blackMs = now;
    if (now - blackMs > BLACK_STOP_MS) { endRun(F("all black (finish box or lifted)")); return; }
  } else blackMs = 0;

  if (nBlack == 0) { lostLine(now); return; }
  if (lostMs) { lostMs = 0; cornerSide = 0; lastErr = err; dFilt = 0; Serial.println(F("[LINE] found again")); }

  bool edgeL = black[0], edgeR = black[NUM_IR - 1];
  if (edgeL != edgeR) { cornerSide = edgeL ? -1 : 1; cornerMs = now; }

  int l, r;
  if (edgeL && edgeR) { l = r = BASE_PWM; }   // junction / cross bar: straight through
  else {
    dFilt += D_FILTER * ((err - lastErr) - dFilt);
    float corr = KP * err + KD * dFilt;
    int speed = BASE_PWM - (int)((1.0 - CURVE_SLOW) * BASE_PWM * abs(err) / CENTRE);
    l = speed + (int)corr;
    r = speed - (int)corr;
  }
  lastErr = err;
  drive(l, r);

  static unsigned long lastPrint = 0;
  if ((telemetry || dry) && now - lastPrint >= 100) {
    lastPrint = now;
    printBars();
    Serial.print(F("pos=")); Serial.print(linePos);
    Serial.print(F(" err=")); Serial.print(err);
    Serial.print(F(" L=")); Serial.print(curL);
    Serial.print(F(" R=")); Serial.println(curR);
  }
}

// ======================= TOOLS =======================
void sensorView() {
  Serial.println(F("Sensor view - any key ends"));
  while (!Serial.available()) {
    readLine();
    printBars();
    for (uint8_t i = 0; i < NUM_IR; i++) { char b[6]; sprintf(b, "%4d ", raw[i]); Serial.print(b); }
    Serial.print(F(" pos=")); Serial.print(linePos);
    Serial.print(F(" black=")); Serial.println(nBlack);
    delay(100);
  }
  while (Serial.available()) Serial.read();
}

void motorTest() {
  Serial.println(F("Motor test - WHEELS OFF THE GROUND. Each wheel should turn as named."));
  const int seq[4][2] = {{100, 0}, {-100, 0}, {0, 100}, {0, -100}};
  const char* const names[4] = {"Left  FORWARD", "Left  BACKWARD", "Right FORWARD", "Right BACKWARD"};
  for (uint8_t k = 0; k < 4; k++) {
    Serial.println(names[k]);
    setMotors(seq[k][0], seq[k][1]); delay(800);
    stopMotors(); delay(400);
  }
  Serial.println(F("Wrong direction? Flip LEFT_REVERSED / RIGHT_REVERSED."));
}

void printHelp() {
  Serial.println(F("\ng=go x=stop d=dry-run t=telemetry s=sensors c=calibrate C=clear-cal m=motors h=help"));
  Serial.println(F("P<kp> D<kd> V<base pwm>   e.g. P0.02 D0.15 V55"));
  Serial.print(F("Kp=")); Serial.print(KP, 4);
  Serial.print(F(" Kd=")); Serial.print(KD, 4);
  Serial.print(F(" base=")); Serial.print(BASE_PWM);
  Serial.println(useCal ? F("  IR: calibrated") : F("  IR: fixed 800/900"));
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  for (uint8_t i = 0; i < NUM_IR; i++) pinMode(IR_PINS[i], INPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT); pinMode(L_EN, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT); pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, HIGH); digitalWrite(R_EN, HIGH);
  stopMotors();

  IrCal c; EEPROM.get(IRCAL_ADDR, c);
  if (c.magic == IRCAL_MAGIC) {
    for (uint8_t i = 0; i < NUM_IR; i++) { calLo[i] = c.lo[i]; calHi[i] = c.hi[i]; }
    useCal = true;
  }

  Serial.println(F("\n=== Line Follower v2 ==="));
  printHelp();

  Serial.println(F("Auto start in 5 s (send any key to cancel)"));
  unsigned long t0 = millis();
  uint8_t shown = 0;
  while (millis() - t0 < START_DELAY_MS) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println(F("Auto start cancelled -> command mode"));
      return;
    }
    uint8_t sec = (millis() - t0) / 1000;
    if (sec >= shown) { Serial.println(START_DELAY_MS / 1000 - sec); shown = sec + 1; }
  }
  startRun(false);
}

void loop() {
  if (running) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == 't') { telemetry = !telemetry; continue; }
      if (c == 'x' || c == 'X' || c == ' ') { running = false; stopMotors(); Serial.println(F("STOPPED")); return; }
    }
    if (millis() - lastLoopMs >= LOOP_MS) { lastLoopMs += LOOP_MS; followStep(); }
    return;
  }

  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'g': startRun(false); break;
    case 'd': startRun(true); break;
    case 'x': stopMotors(); Serial.println(F("STOPPED")); break;
    case 't': telemetry = !telemetry; Serial.print(F("telemetry ")); Serial.println(telemetry); break;
    case 's': sensorView(); break;
    case 'c': calibrate(); break;
    case 'C': clearCalibration(); break;
    case 'm': motorTest(); break;
    case 'P': KP = Serial.parseFloat(); printHelp(); break;
    case 'D': KD = Serial.parseFloat(); printHelp(); break;
    case 'V': BASE_PWM = constrain(Serial.parseInt(), 0, 255); printHelp(); break;
    case 'h': printHelp(); break;
    default: break;
  }
}