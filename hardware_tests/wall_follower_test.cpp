/*
  Maze Wall-Follower v3 — robust version of v2
  Arduino Mega 2560 — EE4360 Maze Solver (25 cm cells)

  Serial Monitor: 115200 baud, line ending "Newline"
  Commands:
    g = GO            x = STOP (any time)        d = dry run (motors OFF)
    z = capture side setpoints (robot CENTERED in a corridor with both walls)
    G = geometry / clearance report             m = motor deadband test (wheels OFF ground)
    e = encoder counts                          F = drive one cell (25 cm)
    < = arc turn LEFT test   > = arc turn RIGHT test   U = 180 spin test
    L = left-hand rule       R = right-hand rule       h = help
  Push button D40 (to GND): press = GO, press again = STOP.

  START POSITION: robot in the start cell, centered, REAR touching the back wall.
  It must face the open side (it can't turn in the start cell).
  FRONT FIRST: a close front wall always stops the robot and triggers a standing wall check.
  DEAD ENDS: no spinning (the robot doesn't fit). It reverses to the first side opening and turns there.
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// ======================= TYPES =======================
enum Rule   { LEFT_HAND, RIGHT_HAND };
enum Action { ACT_STRAIGHT, ACT_LEFT, ACT_RIGHT, ACT_UTURN };
const char* const ACTION_NAME[] = {"STRAIGHT", "TURN LEFT", "TURN RIGHT", "U-TURN"};
struct Walls { bool f; bool l; bool r; };

// ======================= CHASSIS GEOMETRY — MEASURE THESE =======================
const float CELL_CM              = 25.0;
const float HALF_CELL            = 12.5;
const float ROBOT_WIDTH_CM       = 18.5;  // widest point, wheels included
const float AXLE_TO_FRONT_CM     = 15.0;  // wheel-axle centre -> front-most point
const float AXLE_TO_REAR_CM      = 4.0;   // wheel-axle centre -> rear-most point
const float SENSOR_AHEAD_CM      = 15.0;  // wheel-axle centre -> front ultrasonic face
const float SIDE_SENSOR_AHEAD_CM = 11.0;  // wheel-axle centre -> side ultrasonics (forward = +)
const float TRACK_CM             = 16.0;  // left wheel centre -> right wheel centre
const float WHEEL_DIA_CM         = 6.5;
const float GEAR_RATIO           = 34.0;
const float ENC_PPR              = 11.0;
const float WALL_HALF_CM         = 0.5;
const float START_AXLE_FROM_BACK_CM = AXLE_TO_REAR_CM + WALL_HALF_CM;

// ======================= TURNING =======================
float ARC_RADIUS_CM = 12.5;  // axle path radius in a 90° arc (max 12.5)
float ARC_TRIM      = 1.00;  // tune with '<' '>' until the arc ends at exactly 90°
float SPIN_TRIM     = 1.00;  // tune with 'U' until the spin is exactly 180°

// Dead ends: the robot is too long to spin in a corridor, so it reverses out instead
const float REV_LOOK_CM        = 8.0;   // side sensor depth into a cell when checking it while reversing
const uint8_t MAX_REVERSE_CELLS = 8;
const float FRONT_TRUST_CM     = 40.0;  // beyond this the front beam may hit the side walls

// ======================= SPEEDS / GAINS =======================
int BASE_PWM     = 70;
int ARC_PWM      = 65;
int SPIN_PWM     = 55;
int REVERSE_PWM  = 50;
int DEADBAND_PWM = 35;   // lowest PWM that moves a wheel — measure with 'm'
const int MAX_STEER = 30;
const unsigned long RAMP_MS = 250;  // soft start for every move
const int CRUISE_STEP = 5;          // cruise speed-up per side reading

float K_POS  = 0.05;   // wanted heading (rad) per cm off the corridor centre
float K_PSI  = 120.0;  // PWM steer per rad of heading error
const float MAX_PSI = 0.15;        // never aim more than ~9° at a wall
const float SLOPE_BASE_CM = 6.0;   // drive this far between two side readings to measure the wall angle
const float K_FUSE = 0.5;          // how much one wall-angle reading corrects the heading
const float OUTLIER_CM = 2.0;      // raw side reading this far from its median is ignored for the angle
float K_SYNC = 0.3;   // keeps the second wheel locked to the first in moves/turns
int GAP_FIX_PWM = 50;                // follower-only nudge after a move (must beat floor friction)
const float GAP_TOL_CM = 0.3;        // 0.3 cm wheel gap = about 1° heading
const unsigned long GAP_FIX_MS = 600;
const float MAX_ERR_CM = 5.0;

// ======================= ULTRASONIC =======================
const uint8_t NUM_SENSORS = 3;
const uint8_t TRIG_PINS[NUM_SENSORS] = {30, 32, 34};
const uint8_t ECHO_PINS[NUM_SENSORS] = {31, 33, 35};
const uint8_t S_F = 0, S_L = 1, S_R = 2;
const uint8_t PING_ORDER[4] = {S_F, S_L, S_F, S_R};

float airTempC = 30.0;
const unsigned long ECHO_TIMEOUT_US = 6000;
const unsigned long PING_GAP_MS = 30;  // shorter lets one sensor hear another's echo
const float NO_ECHO_CM = 999.0;

float raw[NUM_SENSORS];
float hist[NUM_SENSORS][3];
uint8_t histN[NUM_SENSORS], histI[NUM_SENSORS];
float dist[NUM_SENSORS];               // median of the last 3 readings
uint8_t pingStep = 0;
unsigned long lastPingMs = 0;

struct Calib { uint32_t magic; float gain[NUM_SENSORS]; float offset[NUM_SENSORS]; };
const uint32_t CAL_MAGIC = 0x55533031;
const int CAL_ADDR = 0;
Calib cal;

struct SidePts { uint32_t magic; float lSet; float rSet; };
const uint32_t SP_MAGIC = 0x53505431;
const int SP_ADDR = 64;
float L_SET = 6.0;
float R_SET = 6.0;

// ======================= WALL DETECTION =======================
const float SIDE_MARGIN_CM    = 8.0;
const float FRONT_MARGIN_CM   = 8.0;
const float SIDE_LOOK_MIN_CM  = 4.0;   // side sensor must be this far into the next cell to vote
const float SIDE_LOOK_MAX_CM  = 16.0;
const float FRONT_WINDOW_CM   = 8.0;
const uint8_t MIN_SAMPLES     = 2;
const uint8_t CONFIRM_ROUNDS  = 5;     // standing re-check before any turn
const float FRONT_CORR_MAX_CM = 30.0;
const float FRONT_STOP_MARGIN_CM = 3.0;  // stop this much before the arc start distance
const uint8_t FRONT_STOP_HITS = 2;       // consecutive close front readings needed to stop
const unsigned long STALL_MS  = 400;
const float STALL_MIN_CM      = 0.5;     // less travel than this in STALL_MS = stuck
const float STALL_BACK_CM     = 5.0;

Rule rule = LEFT_HAND;

// ======================= MOTORS (BTS7960 / HW-039) =======================
const uint8_t L_RPWM = 5, L_LPWM = 6, L_EN = 24;
const uint8_t R_RPWM = 7, R_LPWM = 8, R_EN = 25;
const bool LEFT_REVERSED  = false;
const bool RIGHT_REVERSED = true;

// ======================= ENCODERS =======================
const uint8_t L_ENC_A = 2,  L_ENC_B = 3;
const uint8_t R_ENC_A = 18, R_ENC_B = 19;
const bool L_ENC_REVERSED = false;
const bool R_ENC_REVERSED = true;

float COUNTS_PER_CM;

volatile long lCount = 0, rCount = 0;
volatile uint8_t lState = 0, rState = 0;
const int8_t QEM[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

// ======================= STATE =======================
const uint8_t START_BTN = 40;
bool running = false;
bool abortReq = false;
int cruisePwm = 0;

// Heading vs the corridor: encoders track it, side-wall slope corrects it (no gyro)
float psi = 0;                 // rad, + = nose pointing left
float psiTarget = 0;
float odoCm = 0;
struct Anchor { bool ok; float d, s, psi; };
Anchor anc[3];
bool turnCheck = false;

long prevL = 0, prevR = 0;
float toBoundary = 0;          // cm the axle must still travel to the next cell's entry edge
unsigned long lastMoveMs = 0;
long stallRefL = 0, stallRefR = 0;
uint8_t nearF = 0;
uint8_t nF, vF, nL, vL, nR, vR;

// Cell grid for loop detection (start in the middle, heading 0 = start direction)
const uint8_t GRID = 19, G0 = 9;
const int8_t DX[4] = {0, 1, 0, -1};
const int8_t DY[4] = {1, 0, -1, 0};
uint8_t seen[GRID][GRID];
uint8_t gx = G0, gy = G0, heading = 0;

// ======================= ENCODER ISRs =======================
void leftISR() {
  uint8_t ab = (digitalRead(L_ENC_A) << 1) | digitalRead(L_ENC_B);
  lState = ((lState << 2) | ab) & 0x0F;
  lCount += L_ENC_REVERSED ? -QEM[lState] : QEM[lState];
}
void rightISR() {
  uint8_t ab = (digitalRead(R_ENC_A) << 1) | digitalRead(R_ENC_B);
  rState = ((rState << 2) | ab) & 0x0F;
  rCount += R_ENC_REVERSED ? -QEM[rState] : QEM[rState];
}
void getCounts(long &l, long &r) { noInterrupts(); l = lCount; r = rCount; interrupts(); }
void resetCounts() { noInterrupts(); lCount = 0; rCount = 0; interrupts(); }
void syncOdom() { getCounts(prevL, prevR); stallRefL = prevL; stallRefR = prevR; lastMoveMs = millis(); nearF = 0; }

// ======================= MOTORS =======================
void setOne(uint8_t rp, uint8_t lp, bool rev, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (rev) pwm = -pwm;
  if (pwm >= 0) { analogWrite(rp, pwm); analogWrite(lp, 0); }
  else          { analogWrite(rp, 0);   analogWrite(lp, -pwm); }
}
void setMotors(int l, int r) {
  setOne(L_RPWM, L_LPWM, LEFT_REVERSED, l);
  setOne(R_RPWM, R_LPWM, RIGHT_REVERSED, r);
}
void stopMotors() { setMotors(0, 0); }

// ======================= ABORT (serial 'x' or button) =======================
void checkAbort() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X' || c == ' ') { abortReq = true; running = false; }
  }
  if (running && digitalRead(START_BTN) == LOW) {
    delay(30);
    if (digitalRead(START_BTN) == LOW) {
      abortReq = true; running = false;
      stopMotors();
      while (digitalRead(START_BTN) == LOW) {}
      delay(300);
    }
  }
  if (abortReq) stopMotors();
}

// ======================= ULTRASONIC =======================
float med3(float a, float b, float c) { return max(min(a, b), min(max(a, b), c)); }
float soundSpeedCmPerUs() { return (331.3 + 0.606 * airTempC) / 10000.0; }

float readSonar(uint8_t i) {
  digitalWrite(TRIG_PINS[i], LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PINS[i], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PINS[i], LOW);
  unsigned long us = pulseIn(ECHO_PINS[i], HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return NO_ECHO_CM;
  float cm = cal.gain[i] * (us * soundSpeedCmPerUs() / 2.0) + cal.offset[i];
  return (cm < 2.0) ? 2.0 : cm;
}

void resetFilters() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { histN[i] = 0; histI[i] = 0; dist[i] = NO_ECHO_CM; }
}

void pushReading(uint8_t i, float d) {
  raw[i] = d;
  hist[i][histI[i]] = d;
  histI[i] = (histI[i] + 1) % 3;
  if (histN[i] < 3) histN[i]++;
  if (histN[i] == 1)      dist[i] = d;
  else if (histN[i] == 2) dist[i] = max(hist[i][0], hist[i][1]);  // bias to "far" until 3 readings
  else                    dist[i] = med3(hist[i][0], hist[i][1], hist[i][2]);
}

float pingNow(uint8_t i) {
  while (millis() - lastPingMs < PING_GAP_MS) checkAbort();
  float d = readSonar(i);
  lastPingMs = millis();
  pushReading(i, d);
  return d;
}

// Fires the next sensor only when the gap has passed; returns which one, or -1
int8_t pollSonar() {
  if (millis() - lastPingMs < PING_GAP_MS) return -1;
  uint8_t s = PING_ORDER[pingStep];
  pingStep = (pingStep + 1) & 3;
  pingNow(s);
  return s;
}

// tb = distance from axle to the entry edge of the cell whose far wall we test
float frontLimit(float tb) { return tb + CELL_CM - SENSOR_AHEAD_CM - WALL_HALF_CM + FRONT_MARGIN_CM; }
bool sideWallL(float d) { return d < L_SET + SIDE_MARGIN_CM; }
bool sideWallR(float d) { return d < R_SET + SIDE_MARGIN_CM; }
float sideLook() { return SIDE_SENSOR_AHEAD_CM - toBoundary; }

// ---- wall votes (majority over several readings -> one bad echo can't fake a wall or an opening)
void clearVotes() { nF = vF = nL = vL = nR = vR = 0; }
void voteFront(float d, float tb) { if (nF < 250) { nF++; if (d < frontLimit(tb)) vF++; } }
void voteL(float d) { if (nL < 250) { nL++; if (sideWallL(d)) vL++; } }
void voteR(float d) { if (nR < 250) { nR++; if (sideWallR(d)) vR++; } }
Walls votedWalls() {
  Walls w;
  w.f = (vF * 2 > nF);
  w.l = (vL * 2 > nL);
  w.r = (vR * 2 > nR);
  return w;
}

Walls standingWalls(uint8_t rounds, float tb) {
  stopMotors(); delay(150);
  clearVotes();
  for (uint8_t k = 0; k < rounds && !abortReq; k++) {
    voteFront(pingNow(S_F), tb);
    voteL(pingNow(S_L));
    voteR(pingNow(S_R));
  }
  return votedWalls();
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(TRIG_PINS[i], OUTPUT); digitalWrite(TRIG_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT);
  }
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT); pinMode(L_EN, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT); pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, HIGH); digitalWrite(R_EN, HIGH);
  stopMotors();

  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENC_B), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_B), rightISR, CHANGE);

  pinMode(START_BTN, INPUT_PULLUP);

  COUNTS_PER_CM = (ENC_PPR * 4.0 * GEAR_RATIO) / (M_PI * WHEEL_DIA_CM);
  resetFilters();

  EEPROM.get(CAL_ADDR, cal);
  if (cal.magic != CAL_MAGIC) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) { cal.gain[i] = 1.0; cal.offset[i] = 0.0; }
    Serial.println(F("No ultrasonic calibration in EEPROM -> raw readings."));
  } else Serial.println(F("Ultrasonic calibration loaded."));

  SidePts sp; EEPROM.get(SP_ADDR, sp);
  if (sp.magic == SP_MAGIC) { L_SET = sp.lSet; R_SET = sp.rSet; }

  Serial.println(F("\n=== Maze Wall-Follower v3 ==="));
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  abortReq = false;
  switch (c) {
    case 'x': stopMotors(); Serial.println(F("STOPPED")); break;
    case 'e': { long l, r; getCounts(l, r);
                Serial.print(F("L=")); Serial.print(l); Serial.print(F(" R=")); Serial.println(r); } break;
    default: break;
  }
}
