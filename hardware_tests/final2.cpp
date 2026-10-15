/*
  FINAL 2 - Maze -> Line: wall_follower_test.cpp + line_follower_v2.cpp in one run
  The wall follower and line follower code are kept as they are; only the hand-over is added.
  Arduino Mega 2560 — EE4360 Maze Solver (25 cm cells)
  STAGES:
    MAZE   wall follower. The maze floor is measured at the start; while cruising, 6+ IR sensors
           MARKER_DROP whiter than that floor for 3 readings AND no wall within FRONT_TRUST_CM
           ahead (the exit cell is open) = the white marker square.
    CROSS  drive straight over the marker (walls keep it centred) until the IR has left the
           white band and then sees a thin line with white both sides. Wall ahead = false marker.
    LINE   line_follower_v2. Line gone while centred -> drive END_CM; still no line = finished.

  Serial Monitor: 115200 baud, line ending "Newline"
  Commands:
    g = GO            x = STOP (any time)        d = dry run (motors OFF)
    z = capture side setpoints (robot CENTERED in a corridor with both walls)
    G = geometry / clearance report             m = motor deadband test (wheels OFF ground)
    e = encoder counts                          F = drive one cell (25 cm)
    < = arc turn LEFT test   > = arc turn RIGHT test   U = 180 spin test
    L = left-hand rule       R = right-hand rule       h = help
    l = line stage only (robot on the line)   s = IR sensor view   t = line telemetry on/off
    c = IR calibrate (centred ON the line)    C = clear IR calibration (fixed 800/900)
  AUTO START: runs by itself 5 s after power-on. Send any key in those 5 s to stay in command mode.

  START POSITION: robot in the start cell, centered, REAR touching the back wall.
  It must face the open side (it can't turn in the start cell).
  FRONT FIRST: a close front wall always stops the robot and triggers a standing wall check.
  DEAD ENDS: no spinning (the robot doesn't fit). It reverses to the first side opening and turns there.
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// ======================= TYPES =======================
enum Stage  { ST_MAZE, ST_CROSS, ST_LINE };
enum Rule   { LEFT_HAND, RIGHT_HAND };
enum Action { ACT_STRAIGHT, ACT_LEFT, ACT_RIGHT, ACT_UTURN };
const char* const ACTION_NAME[] = {"STRAIGHT", "TURN LEFT", "TURN RIGHT", "U-TURN"};
struct Walls { bool f; bool l; bool r; };

// ======================= CHASSIS GEOMETRY — MEASURE THESE =======================
const float CELL_CM              = 25.0;
const float HALF_CELL            = 12.5;
const float ROBOT_WIDTH_CM       = 18.5;  // widest point, wheels included
const float AXLE_TO_FRONT_CM     = 10.0;  // wheel-axle centre -> front-most point
const float AXLE_TO_REAR_CM      = 6.0;   // wheel-axle centre -> rear-most point
const float SENSOR_AHEAD_CM      = 10.0;  // wheel-axle centre -> front ultrasonic face
const float SIDE_SENSOR_AHEAD_CM = 6.5;   // wheel-axle centre -> side ultrasonics (forward = +)
const float TRACK_CM             = 16.0;  // left wheel centre -> right wheel centre
const float WHEEL_DIA_CM         = 6.5;
const float GEAR_RATIO           = 21.0;  // measured: 25 cm hand push = ~1120 counts
const float ENC_PPR              = 11.0;
const float WALL_HALF_CM         = 0.5;
const float START_AXLE_FROM_BACK_CM = AXLE_TO_REAR_CM + WALL_HALF_CM;

// ======================= TURNING =======================
float ARC_RADIUS_CM = 12.5;  // axle path radius in a 90° arc (max 12.5)
float ARC_TRIM      = 1.00;  // tune with '<' '>' until the arc ends at exactly 90°
float SPIN_TRIM     = 1.00;  // tune with 'U' until the spin is exactly 180°
const float ARC_EARLY_CM = 1.0;  // start arcs this much before the cell edge: centres the corner/post margins

// Dead ends: the robot is too long to spin in a corridor, so it reverses out instead
const float REV_LOOK_CM        = 11.0;  // side sensor depth into a cell when checking it while reversing
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
const uint8_t TRIG_PINS[NUM_SENSORS] = {40, 36, 38};  // front, left, right
const uint8_t ECHO_PINS[NUM_SENSORS] = {41, 37, 39};
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
const float FRONT_STOP_MARGIN_CM = 1.0;  // stop this much before the arc start distance
const uint8_t FRONT_STOP_HITS = 2;       // consecutive close front readings needed to stop
const unsigned long STALL_MS  = 400;
const float STALL_MIN_CM      = 0.5;     // less travel than this in STALL_MS = stuck
const float STALL_BACK_CM     = 5.0;
const unsigned long BOOST_MS  = 150;     // a move with no progress this long gets more power
const int BOOST_STEP = 10, MAX_BOOST = 40;

Rule rule = LEFT_HAND;

// ======================= IR ARRAY (line_follower_v2) =======================
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

// ======================= LINE SPEEDS / GAINS (line_follower_v2) =======================
int LF_BASE_PWM = 55;
int MAX_PWM    = 110;
int SEARCH_PWM = 50;
float KP = 0.020;           // PWM per unit of error (error is -3500..+3500)
float KD = 0.15;            // PWM per unit of error change per loop
const float D_FILTER = 0.5; // smooths the derivative (1 = no smoothing)
const float CURVE_SLOW = 0.5;  // speed at the array edge = 50% of base
const int SLEW = 8;            // max PWM change per loop (soft start, no wheel jerks)
const unsigned long LOOP_MS = 5;  // 200 Hz control loop

const int CENTRE_BAND = 1500;         // line lost within this of centre = gap or line end, not a turn
const unsigned long CORNER_MEMORY_MS = 500;  // a corner seen this recently decides the pivot side
const unsigned long SEARCH_MS = 1500; // give up if the line is not found in this time
const unsigned long BLACK_STOP_MS = 300;

// ======================= MAZE -> LINE HAND-OVER =======================
const uint8_t MARKER_WHITE = 6;      // this many IR sensors much whiter than the maze floor = marker
const int MARKER_DROP = 100;         // raw counts below the floor reading that count as white
const uint8_t MARKER_HITS = 3;       // readings in a row (5 ms apart) before a pattern counts
const uint8_t LINE_MAX_BLACK = 4;    // the 30 mm line covers 3-4 sensors
int CROSS_PWM = 55;
const float CROSS_MAX_CM = 60.0;     // no line this far after the marker -> stop (marker + strip + margin)
const unsigned long CROSS_MAX_MS = 6000;
float END_CM = 15.0;                 // line gone while centred: drive this far, still nothing = finish

// ======================= MOTORS (BTS7960 / HW-039) =======================
const uint8_t L_RPWM = 3, L_LPWM = 2, L_EN = 24;
const uint8_t R_RPWM = 7, R_LPWM = 8, R_EN = 25;
const bool LEFT_REVERSED  = false;
const bool RIGHT_REVERSED = true;

// ======================= ENCODERS =======================
const uint8_t L_ENC_A = 9,  L_ENC_B = 10;  // yellow, green
const uint8_t R_ENC_A = 11, R_ENC_B = 12;
const bool L_ENC_REVERSED = false;
const bool R_ENC_REVERSED = true;

float COUNTS_PER_CM;

volatile long lCount = 0, rCount = 0;
volatile uint8_t rState = 0, lastPB = 0;
const int8_t QEM[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

// ======================= STATE =======================
const unsigned long START_DELAY_MS = 5000;
Stage stage = ST_MAZE;
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
int8_t arcDir = 0;             // last 90° arc: -1 left, +1 right, 0 none (for ARC_TRIM learning)

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

// Line / hand-over state
int irRaw[NUM_IR];
int val[NUM_IR];            // 0 = white .. 1000 = black
bool black[NUM_IR];
uint8_t nBlack = 0;
int linePos = CENTRE;
int lastSeenPos = CENTRE;
int calLo[NUM_IR], calHi[NUM_IR];
bool useCal = false;
bool telemetry = false, waitLine = false, markerFound = false, pastWhite = false;
int floorRaw[NUM_IR];       // maze floor reading per sensor, measured at the start
int curL = 0, curR = 0;
int lastErr = 0;
float dFilt = 0;
unsigned long lostMs = 0, blackMs = 0, lastLoopMs = 0, irMs = 0, crossMs = 0;
float crossCm = 0, lostCm = 0;
uint8_t irHits = 0;
int8_t cornerSide = 0;      // -1 left, +1 right, 0 none: last end sensor that saw the line alone
unsigned long cornerMs = 0;
int8_t searchSide = 0;      // pivot side chosen when the line was lost

// ======================= ENCODER ISRs =======================
// D10/D11/D12 = PB4/PB5/PB6 share one pin-change interrupt; D9 (PH6) has none
ISR(PCINT0_vect) {
  uint8_t pb = PINB;
  uint8_t changed = pb ^ lastPB;
  lastPB = pb;
  if (changed & _BV(PB4)) {
    // left counts on green edges only, x2 to keep the same counts/cm
    bool a = PINH & _BV(PH6), b = pb & _BV(PB4);
    int8_t d = (a != b) ? -2 : 2;
    lCount += L_ENC_REVERSED ? -d : d;
  }
  if (changed & (_BV(PB5) | _BV(PB6))) {
    uint8_t ab = (((pb >> PB5) & 1) << 1) | ((pb >> PB6) & 1);
    rState = ((rState << 2) | ab) & 0x0F;
    rCount += R_ENC_REVERSED ? -QEM[rState] : QEM[rState];
  }
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
  curL = l; curR = r;
  setOne(L_RPWM, L_LPWM, LEFT_REVERSED, l);
  setOne(R_RPWM, R_LPWM, RIGHT_REVERSED, r);
}
void stopMotors() { setMotors(0, 0); }

// ======================= ABORT (serial 'x') =======================
void checkAbort() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') telemetry = !telemetry;
    if (c == 'x' || c == 'X' || c == ' ') { abortReq = true; running = false; }
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

// ======================= GEOMETRY =======================
float frontCornerRadius() { return sqrt(sq(AXLE_TO_FRONT_CM) + sq(ROBOT_WIDTH_CM / 2.0)); }
float rearCornerRadius()  { return sqrt(sq(AXLE_TO_REAR_CM)  + sq(ROBOT_WIDTH_CM / 2.0)); }
// How far past the cell edge the axle must go before the side sensors see that cell
float decisionOffset()    { float o = SIDE_LOOK_MIN_CM + 2.0 - SIDE_SENSOR_AHEAD_CM; return (o > 0) ? o : 0; }
float arcRadius()         { return min(ARC_RADIUS_CM, HALF_CELL - decisionOffset()); }
float arcFrontTarget(float R) { return HALF_CELL + R + ARC_EARLY_CM - SENSOR_AHEAD_CM - WALL_HALF_CM; }
float frontStopCm()       { return arcFrontTarget(arcRadius()) + FRONT_STOP_MARGIN_CM; }
// Where the axle stops inside a cell (from its dead-end side) while reversing: sensors see it, rear stays clear
float reverseStop()       { return min(SIDE_SENSOR_AHEAD_CM + REV_LOOK_CM, CELL_CM - AXLE_TO_REAR_CM - WALL_HALF_CM - 1.0); }
float reverseOutRadius()  { return max(reverseStop() - HALF_CELL, 0.0); }

void geometryReport() {
  float hw = ROBOT_WIDTH_CM / 2.0;
  float R = arcRadius();
  float outerCorner = sqrt(sq(R + hw) + sq(AXLE_TO_FRONT_CM));
  float outerClear  = (HALF_CELL + R) - outerCorner - WALL_HALF_CM + ARC_EARLY_CM;
  float innerClear  = (R - hw) - sqrt(2.0) * (HALF_CELL - R) - WALL_HALF_CM - ARC_EARLY_CM;
  float spinClear   = HALF_CELL - WALL_HALF_CM - max(frontCornerRadius(), rearCornerRadius());
  float ratio       = (R - TRACK_CM / 2.0) / (R + TRACK_CM / 2.0);

  Serial.println(F("\n--- Turn geometry (25 cm cell) ---"));
  Serial.print(F("Decision point: axle ")); Serial.print(decisionOffset(), 1);
  Serial.println(F(" cm past the cell edge (side sensors must see the new cell)"));
  Serial.print(F("Arc radius ")); Serial.print(R, 1);
  Serial.print(F(" cm -> inner wheel at ")); Serial.print(ratio * 100.0, 0);
  Serial.println(F("% of outer wheel"));
  Serial.print(F("Arc: outer corner clearance to far wall = ")); Serial.print(outerClear, 1);
  Serial.println(outerClear > 0 ? F(" cm  OK") : F(" cm  WARNING: will clip the far wall"));
  Serial.print(F("Arc: inner clearance to wall corner     = ")); Serial.print(innerClear, 1);
  Serial.println(innerClear > 0 ? F(" cm  OK") : F(" cm  WARNING: will clip the inner corner"));
  Serial.print(F("Spin: corner clearance to side walls    = ")); Serial.print(spinClear, 1);
  Serial.println(spinClear > 0 ? F(" cm  OK") : F(" cm  WARNING: in-place spins (start/recovery) will scrape"));
  Serial.print(F("Dead end: reverse out, then turn with radius ")); Serial.print(reverseOutRadius(), 1);
  Serial.println(F(" cm"));
}

// ======================= MOTION PRIMITIVES =======================
// Heading depends only on (left - right) wheel travel. After a move, nudge the follower wheel
// until that gap matches the plan, so lag or coasting can't leave the robot angled.
void fixWheelGap(long gapTarget, bool leadLeft) {
  long tol = lround(GAP_TOL_CM * COUNTS_PER_CM);
  unsigned long t0 = millis();
  while (!abortReq && millis() - t0 < GAP_FIX_MS) {
    checkAbort();
    long l, r; getCounts(l, r);
    long need = (l - r) - gapTarget;     // follower=right: move it +need; follower=left: move it -need
    if (!leadLeft) need = -need;
    if (labs(need) <= tol) break;
    int p = (need > 0) ? GAP_FIX_PWM : -GAP_FIX_PWM;
    if (leadLeft) setMotors(0, p); else setMotors(p, 0);
  }
  stopMotors();
  delay(60);
}

// Moves each wheel a signed distance (cm) with soft start/stop.
// The wheel with the shorter path is locked to the longer one by the encoders.
bool wheelMove(float dL, float dR, int pwm, unsigned long timeoutMs) {
  long tL = lround(dL * COUNTS_PER_CM), tR = lround(dR * COUNTS_PER_CM);
  bool leadLeft = labs(tL) >= labs(tR);
  long tLead = leadLeft ? tL : tR;
  long tFol  = leadLeft ? tR : tL;
  if (tLead == 0) return true;
  float ratio = (float)tFol / tLead;
  int sLead = (tLead > 0) ? 1 : -1;
  long target = labs(tLead);
  long slow = max(target / 4, (long)(3 * COUNTS_PER_CM));

  resetCounts();
  unsigned long t0 = millis(), progMs = t0;
  long progLead = 0;
  int boost = 0;
  bool done = false;
  while (!abortReq) {
    checkAbort();
    long l, r; getCounts(l, r);
    long lead = (leadLeft ? l : r) * sLead;
    long fol  = (leadLeft ? r : l) * sLead;
    if (lead >= target) { done = true; break; }
    unsigned long el = millis() - t0;
    if (el > timeoutMs) break;
    // no progress: more power first (floor friction), give up only at full boost (against a wall)
    if (lead - progLead > STALL_MIN_CM * COUNTS_PER_CM) { progLead = lead; progMs = millis(); }
    else if (el > RAMP_MS && millis() - progMs > BOOST_MS) {
      if (boost < MAX_BOOST) { boost += BOOST_STEP; progMs = millis(); }
      else { Serial.println(F("[MOVE] stuck -> stop")); break; }
    }

    int sp = pwm;
    if (el < RAMP_MS) sp = DEADBAND_PWM + (int)((pwm - DEADBAND_PWM) * (float)el / RAMP_MS);
    long left = target - lead;
    if (left < slow) sp = min(sp, DEADBAND_PWM + 10 + (int)((pwm - DEADBAND_PWM - 10) * (float)left / slow));
    sp = min(sp + boost, 200);

    float err = ratio * lead - fol;
    float ff  = (fabs(ratio) > 0.05) ? (DEADBAND_PWM + (sp - DEADBAND_PWM) * fabs(ratio)) * (ratio > 0 ? 1 : -1) : 0;
    int fp = constrain((int)(ff + K_SYNC * err), -200, 200);
    int lp = max(sp - constrain((int)(0.5 * K_SYNC * fabs(err)), 0, 25), DEADBAND_PWM + 5);

    if (leadLeft) setMotors(sLead * lp, sLead * fp);
    else          setMotors(sLead * fp, sLead * lp);
  }
  stopMotors();
  delay(100);
  if (done) fixWheelGap(tL - tR, leadLeft);
  return done;
}

void driveCm(float cm) { wheelMove(cm, cm, BASE_PWM, 4000); }
void updateOdom();

// dir = -1 left, +1 right
bool arcTurn(int dir, float R, float deg = 90.0) {
  float outer = (R + TRACK_CM / 2.0) * radians(deg) * ARC_TRIM;
  float inner = (R - TRACK_CM / 2.0) * radians(deg) * ARC_TRIM;
  if (dir < 0) return wheelMove(inner, outer, ARC_PWM, 5000);
  return wheelMove(outer, inner, ARC_PWM, 5000);
}

// deg > 0 = right (clockwise)
void spinDeg(float deg) {
  float d = M_PI * TRACK_CM * fabs(deg) / 360.0 * SPIN_TRIM;
  if (deg > 0) wheelMove(d, -d, SPIN_PWM, 5000);
  else         wheelMove(-d, d, SPIN_PWM, 5000);
}

// Move forward/back until the front sensor reads targetCm. Gives up if the wall echo is lost.
void approachFront(float targetCm) {
  unsigned long t0 = millis();
  uint8_t ok = 0, lost = 0;
  while (!abortReq && millis() - t0 < 3000) {
    checkAbort();
    float err = pingNow(S_F) - targetCm;
    if (err > 15.0) { stopMotors(); if (++lost >= 3) break; continue; }
    lost = 0;
    if (fabs(err) < 0.8) { stopMotors(); if (++ok >= 3) break; continue; }
    ok = 0;
    int sp = constrain((int)(DEADBAND_PWM + 4.0 * fabs(err)), DEADBAND_PWM + 5, 60);
    if (err > 0) setMotors(sp, sp); else setMotors(-sp, -sp);
  }
  stopMotors();
  delay(100);
}

// ======================= DECISION =======================
Action decide(Walls w) {
  bool prefOpen  = (rule == LEFT_HAND) ? !w.l : !w.r;
  bool otherOpen = (rule == LEFT_HAND) ? !w.r : !w.l;
  Action prefTurn  = (rule == LEFT_HAND) ? ACT_LEFT  : ACT_RIGHT;
  Action otherTurn = (rule == LEFT_HAND) ? ACT_RIGHT : ACT_LEFT;
  if (prefOpen)  return prefTurn;
  if (!w.f)      return ACT_STRAIGHT;
  if (otherOpen) return otherTurn;
  return ACT_UTURN;
}

void logDecision(const __FlashStringHelper* tag, Walls w, Action a) {
  Serial.print(tag);
  Serial.print(F(" votes F")); Serial.print(vF); Serial.print('/'); Serial.print(nF);
  Serial.print(F(" L")); Serial.print(vL); Serial.print('/'); Serial.print(nL);
  Serial.print(F(" R")); Serial.print(vR); Serial.print('/'); Serial.print(nR);
  Serial.print(F("  walls F")); Serial.print(w.f); Serial.print(F(" L")); Serial.print(w.l);
  Serial.print(F(" R")); Serial.print(w.r);
  Serial.print(F(" -> ")); Serial.println(ACTION_NAME[a]);
}

void turnHeading(Action a) {
  if (a == ACT_LEFT)  heading = (heading + 3) & 3;
  if (a == ACT_RIGHT) heading = (heading + 1) & 3;
  if (a == ACT_UTURN) heading = (heading + 2) & 3;
}

// A wall follower that enters the same cell with the same heading twice is in a loop
void enterCell() {
  gx = constrain(gx + DX[heading], 0, GRID - 1);
  gy = constrain(gy + DY[heading], 0, GRID - 1);
  uint8_t bit = 1 << heading;
  if (seen[gx][gy] & bit) {
    rule = (rule == LEFT_HAND) ? RIGHT_HAND : LEFT_HAND;
    memset(seen, 0, sizeof(seen));
    Serial.println(F("[LOOP] been here before -> switching hand rule"));
  }
  seen[gx][gy] |= bit;
}

void resumeCruise() {
  cruisePwm = DEADBAND_PWM + 10;
  for (uint8_t i = 0; i < 3; i++) anc[i].ok = false;
  syncOdom();
}

// After a turn we assume it ended straight; the first wall-angle reading corrects whatever is left
void afterManeuver() {
  psi = 0; psiTarget = 0; turnCheck = true;
  resetFilters();
  clearVotes();
  resumeCruise();
}

// Dead end: reverse straight back cell by cell until a side opens, then turn into it
void reverseOut() {
  float stop = reverseStop();
  float f = med3(pingNow(S_F), pingNow(S_F), pingNow(S_F));
  float a = (f < FRONT_TRUST_CM) ? f + SENSOR_AHEAD_CM + WALL_HALF_CM : toBoundary + CELL_CM;  // axle -> dead-end wall

  for (uint8_t k = 1; k <= MAX_REVERSE_CELLS && !abortReq; k++) {
    float target = k * CELL_CM + stop;
    if (target > a) wheelMove(a - target, a - target, REVERSE_PWM, 4000);
    a = target;
    float frontTarget = target - SENSOR_AHEAD_CM - WALL_HALF_CM;
    if (frontTarget < FRONT_TRUST_CM) approachFront(frontTarget);

    Walls w = standingWalls(CONFIRM_ROUNDS, 0);
    if (abortReq) return;
    if (w.l && w.r) continue;

    // both open: take the side the hand rule would pick after turning around
    Action t;
    if (!w.l && !w.r) t = (rule == LEFT_HAND) ? ACT_RIGHT : ACT_LEFT;
    else t = w.l ? ACT_RIGHT : ACT_LEFT;
    logDecision(F("[REVERSE-OUT]"), w, t);

    gx = constrain(gx - k * DX[heading], 0, GRID - 1);
    gy = constrain(gy - k * DY[heading], 0, GRID - 1);
    float R = reverseOutRadius();
    arcTurn(t == ACT_LEFT ? -1 : 1, R);
    toBoundary = HALF_CELL - R;
    turnHeading(t);
    return;
  }
  Serial.println(F("[REVERSE-OUT] no opening found -> stopping"));
  abortReq = true; running = false;
  stopMotors();
}

// Turn from where the robot stands (start, recovery). 90° spins assume it is near the cell centre.
void pivot(Action a) {
  if (a == ACT_UTURN) { reverseOut(); return; }
  spinDeg(a == ACT_LEFT ? -90 : 90);
  toBoundary = HALF_CELL;
  turnHeading(a);
}

// Stop, recheck all walls standing still, then act. frontWall = the front sensor already saw a wall.
void confirmAndAct(bool frontWall) {
  Walls w = standingWalls(CONFIRM_ROUNDS, toBoundary);
  if (frontWall) w.f = true;
  Action a = decide(w);
  logDecision(F("[CONFIRM]"), w, a);
  if (abortReq) return;
  updateOdom();   // count the roll after stopping, or the turn starts late

  if (a == ACT_STRAIGHT) {
    toBoundary += CELL_CM;
    clearVotes();
    resumeCruise();
    return;
  }
  if (a == ACT_LEFT || a == ACT_RIGHT) {
    float R = arcRadius();
    int dir = (a == ACT_LEFT) ? -1 : 1;
    if (w.f) approachFront(arcFrontTarget(R));
    else {
      float pre = (HALF_CELL - R) + toBoundary - ARC_EARLY_CM;
      if (fabs(pre) > 0.3) driveCm(pre);   // negative = rolled past the start point, back up
    }
    // already angled toward the turn -> turn less, angled away -> turn more
    float deg = constrain(90.0 + dir * degrees(psi), 75.0, 105.0);
    float psi0 = psi;
    bool ok = abortReq || arcTurn(dir, R, deg);
    long l, r; getCounts(l, r);
    toBoundary = HALF_CELL - R;
    turnHeading(a);
    afterManeuver();
    arcDir = ok ? dir : 0;
    if (!ok) {
      // arc blocked part way: keep the real angle so steering finishes the turn instead of driving into the wall
      psi = psi0 + (r - l) / COUNTS_PER_CM / TRACK_CM + dir * M_PI / 2.0;
      Serial.print(F("[TURN] arc blocked, ")); Serial.print(degrees(psi), 0); Serial.println(F(" deg left to fix"));
    }
  } else {
    pivot(a);
    afterManeuver();
    arcDir = 0;
  }
}

// Called when odometry says the side sensors can see into the next cell
void atDecisionPoint() {
  enterCell();
  bool enough = nF >= MIN_SAMPLES && nL >= MIN_SAMPLES && nR >= MIN_SAMPLES;
  Walls w = votedWalls();
  Action a = enough ? decide(w) : ACT_UTURN;
  if (enough) logDecision(F("[CELL]"), w, a);
  if (a != ACT_STRAIGHT) { confirmAndAct(false); return; }
  toBoundary += CELL_CM;
  clearVotes();
}

// Front sensor sees a close wall: it wins over odometry and side readings, decide right here
void frontStop(float f) {
  stopMotors();
  Serial.println(F("[FRONT] wall close -> stop and decide"));
  if (toBoundary > 0) enterCell();   // odometry hadn't reached this cell yet
  toBoundary = f + SENSOR_AHEAD_CM + WALL_HALF_CM - CELL_CM;
  confirmAndAct(true);
}

// Wheels stuck (scraping a wall): back off, then let the front sensor decide
void recoverFromStall() {
  Serial.println(F("[STALL] wheels stuck -> backing off"));
  wheelMove(-STALL_BACK_CM, -STALL_BACK_CM, REVERSE_PWM, 1500);
  prevL = prevR = 0;   // wheelMove zeroed the counts
  updateOdom();
  float f = med3(pingNow(S_F), pingNow(S_F), pingNow(S_F));
  if (f < frontStopCm()) frontStop(f);
  else { resetFilters(); resumeCruise(); }
}

// ======================= WALL FOLLOWING =======================
// Wall gyro: the change in side distance per cm driven gives the robot's angle to that wall
void wallAngle(uint8_t s) {
  Anchor &a = anc[s];
  float d = raw[s];
  bool wall = (s == S_L) ? sideWallL(d) : sideWallR(d);
  if (!wall || d <= 2.0 || histN[s] < 2 || fabs(d - dist[s]) > OUTLIER_CM) { a.ok = false; return; }
  if (!a.ok) { a = {true, d, odoCm, psi}; return; }
  float ds = odoCm - a.s;
  if (ds < SLOPE_BASE_CM) return;

  float dy = (s == S_R) ? (d - a.d) : (a.d - d);    // sideways move of the sensor, + = left
  dy -= SIDE_SENSOR_AHEAD_CM * (psi - a.psi);       // remove the part caused by rotating
  float err = atan2(dy, ds) - (psi + a.psi) / 2.0;  // wall says vs encoders say (window average)
  if (turnCheck) {
    Serial.print(F("[TURN] ended ")); Serial.print(degrees(err), 1); Serial.println(F(" deg off (+ = too far left)"));
    if (arcDir) {
      // learn from each turn: over-turned -> shorter arcs next time
      float over = constrain(arcDir < 0 ? degrees(err) : -degrees(err), -10.0, 10.0);
      ARC_TRIM = constrain(ARC_TRIM * (1.0 - 0.5 * over / 90.0), 0.85, 1.10);
      Serial.print(F("[TURN] ARC_TRIM now ")); Serial.println(ARC_TRIM, 3);
    }
    turnCheck = false;
  }
  psi += K_FUSE * err;
  a = {true, d, odoCm, psi};
}

// Off-centre -> aim a few degrees back toward the centre line (no walls -> aim straight)
void updateTarget() {
  if (cruisePwm < BASE_PWM) cruisePwm = min(BASE_PWM, cruisePwm + CRUISE_STEP);
  bool lW = histN[S_L] >= 2 && sideWallL(dist[S_L]);
  bool rW = histN[S_R] >= 2 && sideWallR(dist[S_R]);
  float err = 0;                                    // + = too far right
  if (lW && rW)  err = ((dist[S_L] - L_SET) - (dist[S_R] - R_SET)) / 2.0;
  else if (lW)   err = dist[S_L] - L_SET;
  else if (rW)   err = R_SET - dist[S_R];
  err = constrain(err, -MAX_ERR_CM, MAX_ERR_CM);
  psiTarget = constrain(K_POS * err, -MAX_PSI, MAX_PSI);
}

void steer() {
  float st = constrain(K_PSI * (psiTarget - psi), -MAX_STEER, MAX_STEER);
  setMotors(cruisePwm - st, cruisePwm + st);
}

void startLine(bool wait);

// ======================= IR ARRAY =======================
int readIr(uint8_t i) { return analogRead(IR_PINS[IR_REVERSE_ORDER ? NUM_IR - 1 - i : i]); }

void readLine() {
  long weighted = 0, sum = 0;
  nBlack = 0;
  for (uint8_t i = 0; i < NUM_IR; i++) {
    irRaw[i] = readIr(i);
    int lo = useCal ? calLo[i] : WHITE_MAX;
    int hi = useCal ? calHi[i] : BLACK_MIN;
    val[i] = constrain((long)(irRaw[i] - lo) * 1000L / (hi - lo), 0L, 1000L);
    if (val[i] >= ON_LINE) black[i] = true;          // in between keeps the old state
    else if (val[i] <= OFF_LINE) black[i] = false;
    if (black[i]) nBlack++;
    if (val[i] < NOISE) continue;
    weighted += (long)val[i] * i * 1000L;
    sum += val[i];
  }
  if (nBlack > 0 && sum > 0) { linePos = weighted / sum; lastSeenPos = linePos; }
}

// Thin line with white on both sides: 1..LINE_MAX_BLACK neighbouring sensors, end sensors white
bool linePattern() {
  if (nBlack == 0 || nBlack > LINE_MAX_BLACK || black[0] || black[NUM_IR - 1]) return false;
  int8_t first = -1, last = -1;
  for (uint8_t i = 0; i < NUM_IR; i++) if (black[i]) { if (first < 0) first = i; last = i; }
  return last - first + 1 == nBlack;
}

void printBars() {
  Serial.print('|');
  for (uint8_t i = 0; i < NUM_IR; i++) Serial.print(black[i] ? '#' : '.');
  Serial.print(F("| "));
}

// ======================= STAGE 1 -> 2: MARKER =======================
// Sensors reading clearly whiter than the maze floor (works on a shiny floor, no fixed numbers)
uint8_t whiteCount() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < NUM_IR; i++) if (floorRaw[i] - irRaw[i] >= MARKER_DROP) n++;
  return n;
}

void measureFloor() {
  long sum[NUM_IR] = {0};
  for (uint8_t k = 0; k < 8; k++) {
    for (uint8_t i = 0; i < NUM_IR; i++) sum[i] += readIr(i);
    delay(5);
  }
  Serial.print(F("[MARKER] maze floor:"));
  for (uint8_t i = 0; i < NUM_IR; i++) { floorRaw[i] = sum[i] / 8; Serial.print(' '); Serial.print(floorRaw[i]); }
  Serial.print(F("  -> marker = 6+ sensors below floor - ")); Serial.println(MARKER_DROP);
}

// Every LOOP_MS while cruising in the maze
void markerCheck() {
  if (!running || stage != ST_MAZE || markerFound || millis() - irMs < LOOP_MS) return;
  irMs = millis();
  readLine();
  irHits = (whiteCount() >= MARKER_WHITE) ? irHits + 1 : 0;
  if (irHits >= MARKER_HITS) markerFound = true;
}

void startCross() {
  stage = ST_CROSS;
  crossCm = odoCm; crossMs = millis();
  pastWhite = false; irHits = 0;
  cruisePwm = CROSS_PWM;
  nearF = 0;
  Serial.println(F("[MARKER] white square seen -> crossing it"));
}

// Straight over the marker: wall centring steers, the IR only waits for black, then the line
void crossStep() {
  updateOdom();
  int8_t s = pollSonar();
  if (s == S_L || s == S_R) { wallAngle(s); updateTarget(); }
  if (s == S_F) {
    nearF = (raw[S_F] < frontStopCm()) ? nearF + 1 : 0;
    if (nearF >= FRONT_STOP_HITS) {
      Serial.println(F("[MARKER] wall ahead -> false marker, back to maze"));
      stage = ST_MAZE; markerFound = false; irHits = 0;
      frontStop(raw[S_F]);
      return;
    }
  }
  cruisePwm = CROSS_PWM;
  steer();

  if (odoCm - crossCm > CROSS_MAX_CM || millis() - crossMs > CROSS_MAX_MS) {
    Serial.println(F("[MARKER] no line after the marker -> stopping"));
    abortReq = true; running = false;
    return;
  }
  if (millis() - irMs < LOOP_MS) return;
  irMs = millis();
  readLine();
  if (whiteCount() <= NUM_IR - MARKER_WHITE) pastWhite = true;   // off the white band
  irHits = (pastWhite && linePattern()) ? irHits + 1 : 0;
  if (irHits >= MARKER_HITS) startLine(false);
}

// ======================= STAGE 3: LINE (line_follower_v2) =======================
// Slew-limited drive
void drive(int l, int r) {
  l = constrain(constrain(l, -MAX_PWM, MAX_PWM), curL - SLEW, curL + SLEW);
  r = constrain(constrain(r, -MAX_PWM, MAX_PWM), curR - SLEW, curR + SLEW);
  setMotors(l, r);
}

void startLine(bool wait) {
  stage = ST_LINE;
  running = true; abortReq = false;
  dFilt = 0; lostMs = 0; blackMs = 0;
  lastErr = linePos - CENTRE;
  lastSeenPos = wait ? CENTRE : linePos;
  cornerSide = 0; searchSide = 0;
  waitLine = wait;
  syncOdom();
  lastLoopMs = millis();
  Serial.println(wait ? F("[LINE] waiting for the line") : F("[LINE] line found -> following"));
}

void endRun(const __FlashStringHelper* why) {
  running = false;
  stopMotors();
  Serial.print(F("STOP: ")); Serial.println(why);
}

// Line missing: recent corner -> pivot that way; centred -> straight, no line after END_CM = finish;
// else pivot toward the side it was last seen
void lostLine(unsigned long now) {
  if (lostMs == 0) {
    lostMs = now; lostCm = odoCm;
    searchSide = (cornerSide != 0 && now - cornerMs < CORNER_MEMORY_MS) ? cornerSide : 0;
    Serial.print(F("[LINE] lost, last pos ")); Serial.print(lastSeenPos);
    if (searchSide < 0) Serial.println(F(" -> corner LEFT"));
    else if (searchSide > 0) Serial.println(F(" -> corner RIGHT"));
    else Serial.println();
  }
  unsigned long t = now - lostMs;
  int8_t side = searchSide;
  if (side == 0 && abs(lastSeenPos - CENTRE) < CENTRE_BAND) {
    if (odoCm - lostCm > END_CM || t > SEARCH_MS) { endRun(F("FINISH - line ended")); return; }
    drive(LF_BASE_PWM, LF_BASE_PWM);
    return;
  }
  if (t > SEARCH_MS) { endRun(F("line lost")); return; }
  if (side == 0) side = (lastSeenPos < CENTRE) ? -1 : 1;
  drive(side * SEARCH_PWM, -side * SEARCH_PWM);
}

void followStep() {
  updateOdom();
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
  if (edgeL && edgeR) { l = r = LF_BASE_PWM; }   // junction / cross bar: straight through
  else {
    dFilt += D_FILTER * ((err - lastErr) - dFilt);
    float corr = KP * err + KD * dFilt;
    int speed = LF_BASE_PWM - (int)((1.0 - CURVE_SLOW) * LF_BASE_PWM * abs(err) / CENTRE);
    l = speed + (int)corr;
    r = speed - (int)corr;
  }
  lastErr = err;
  drive(l, r);

  static unsigned long lastPrint = 0;
  if (telemetry && now - lastPrint >= 100) {
    lastPrint = now;
    printBars();
    Serial.print(F("pos=")); Serial.print(linePos);
    Serial.print(F(" err=")); Serial.print(err);
    Serial.print(F(" L=")); Serial.print(curL);
    Serial.print(F(" R=")); Serial.println(curR);
  }
}

// ======================= MAIN STEP =======================
void startRun() {
  running = true; abortReq = false;
  memset(seen, 0, sizeof(seen));
  gx = G0; gy = G0; heading = 0;
  seen[G0][G0] = 1;
  toBoundary = CELL_CM - START_AXLE_FROM_BACK_CM;
  resetFilters();

  Walls w = standingWalls(CONFIRM_ROUNDS, toBoundary - CELL_CM);
  if (w.f && !abortReq) {
    logDecision(F("[START]"), w, decide(w));
    Serial.println(F("[START] facing a wall - can't turn in the start cell. Place it facing the open side."));
    abortReq = true; running = false; stopMotors();
    return;
  }
  afterManeuver();
  stage = ST_MAZE;
  irHits = 0; markerFound = false;
  measureFloor();
  Serial.println(F("GO"));
}

void updateOdom() {
  long l, r; getCounts(l, r);
  float move = ((l - prevL) + (r - prevR)) / 2.0 / COUNTS_PER_CM;
  toBoundary -= move;
  odoCm += move;
  psi += ((r - prevR) - (l - prevL)) / COUNTS_PER_CM / TRACK_CM;
  prevL = l; prevR = r;
}

void mazeStep() {
  markerCheck();
  if (markerFound) {
    // the real marker is the exit cell: nothing ahead. A close wall means a floor glint.
    if (dist[S_F] >= FRONT_TRUST_CM) { startCross(); return; }
    Serial.print(F("[MARKER] ignored, wall ahead at ")); Serial.println(dist[S_F], 1);
    markerFound = false; irHits = 0;
  }
  long l, r; getCounts(l, r);
  if (cruisePwm < BASE_PWM) lastMoveMs = millis();   // still speeding up: too early to call it stuck
  if (labs((l - stallRefL) + (r - stallRefR)) > 2.0 * STALL_MIN_CM * COUNTS_PER_CM) {
    stallRefL = l; stallRefR = r; lastMoveMs = millis();
  }
  updateOdom();

  int8_t s = pollSonar();
  if (s == S_F) {
    nearF = (raw[S_F] < frontStopCm()) ? nearF + 1 : 0;
    if (nearF >= FRONT_STOP_HITS) { frontStop(raw[S_F]); return; }
    if (toBoundary <= FRONT_WINDOW_CM) voteFront(raw[S_F], toBoundary);

    // walls sit on cell edges, so a front wall tells us where we are in the cell
    if (raw[S_F] < FRONT_CORR_MAX_CM) {
      float wallDist = raw[S_F] + SENSOR_AHEAD_CM + WALL_HALF_CM;
      long k = lround((wallDist - toBoundary) / CELL_CM);
      if (k < 0) k = 0;
      float tb = wallDist - k * CELL_CM;
      if (fabs(tb - toBoundary) < 6.0) toBoundary += 0.5 * (tb - toBoundary);
    }
  } else if (s == S_L || s == S_R) {
    float look = sideLook();
    if (look >= SIDE_LOOK_MIN_CM && look <= SIDE_LOOK_MAX_CM) {
      if (s == S_L) voteL(raw[S_L]); else voteR(raw[S_R]);
    }
    wallAngle(s);
    updateTarget();
  }
  steer();

  if (millis() - lastMoveMs > STALL_MS) { recoverFromStall(); return; }
  if (toBoundary <= -decisionOffset()) atDecisionPoint();
}

// ======================= TOOLS =======================
void sensorView() {
  Serial.println(F("IR sensor view - any key ends"));
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    readLine();
    printBars();
    for (uint8_t i = 0; i < NUM_IR; i++) { char b[6]; sprintf(b, "%4d ", irRaw[i]); Serial.print(b); }
    Serial.print(F(" pos=")); Serial.print(linePos);
    Serial.print(F(" black=")); Serial.print(nBlack);
    Serial.println(linePattern() ? F("  LINE") : F(""));
    delay(100);
  }
  while (Serial.available()) Serial.read();
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
void dryRun() {
  Serial.println(F("\nDry run (motors OFF) — press any key + Enter to stop."));
  while (Serial.available()) Serial.read();
  resetFilters();
  unsigned long lastPrint = 0;
  while (!Serial.available()) {
    pollSonar();
    if (millis() - lastPrint < 200) continue;
    lastPrint = millis();
    Walls w = { dist[S_F] < frontLimit(0), sideWallL(dist[S_L]), sideWallR(dist[S_R]) };
    Serial.print(F("F=")); Serial.print(dist[S_F], 1);
    Serial.print(F(" L=")); Serial.print(dist[S_L], 1);
    Serial.print(F(" R=")); Serial.print(dist[S_R], 1);
    Serial.print(F("  walls F")); Serial.print(w.f); Serial.print(F(" L")); Serial.print(w.l);
    Serial.print(F(" R")); Serial.print(w.r);
    Serial.print(F(" -> ")); Serial.println(ACTION_NAME[decide(w)]);
  }
  while (Serial.available()) Serial.read();
}

void captureSetpoints() {
  Serial.println(F("Capturing side setpoints — robot must be CENTERED in a corridor."));
  float ls = 0, rs = 0; uint8_t n = 0;
  for (uint8_t k = 0; k < 10; k++) {
    float l = pingNow(S_L), r = pingNow(S_R);
    if (l < 20.0 && r < 20.0) { ls += l; rs += r; n++; }
  }
  if (n < 5) { Serial.println(F("Both side walls not seen reliably. Not saved.")); return; }
  L_SET = ls / n; R_SET = rs / n;
  SidePts sp = {SP_MAGIC, L_SET, R_SET};
  EEPROM.put(SP_ADDR, sp);
  Serial.print(F("Saved L_SET=")); Serial.print(L_SET, 2);
  Serial.print(F(" R_SET=")); Serial.println(R_SET, 2);
}

void deadbandTest() {
  Serial.println(F("\nDeadband test — WHEELS OFF THE GROUND."));
  const char* const wn[2] = {"Left", "Right"};
  int worst = 0;
  for (uint8_t w = 0; w < 2; w++) {
    for (int8_t d = 1; d >= -1; d -= 2) {
      resetCounts();
      int found = -1;
      for (int pwm = 0; pwm <= 150; pwm += 2) {
        if (w == 0) setMotors(d * pwm, 0); else setMotors(0, d * pwm);
        delay(120);
        long l, r; getCounts(l, r);
        if (labs(w == 0 ? l : r) > 20) { found = pwm; break; }
      }
      stopMotors(); delay(400);
      Serial.print(wn[w]); Serial.print(d > 0 ? F(" fwd: ") : F(" rev: "));
      if (found < 0) Serial.println(F("did not move up to 150 (check wiring/encoder)"));
      else { Serial.print(found); Serial.println(F(" PWM")); if (found > worst) worst = found; }
    }
  }
  Serial.print(F("Set DEADBAND_PWM to about ")); Serial.println(worst);
}

void printHelp() {
  Serial.println(F("\ng=go x=stop d=dry-run z=setpoints G=geometry m=deadband e=encoders"));
  Serial.println(F("F=1 cell  <=arc left  >=arc right  U=180 spin  L/R=hand rule  h=help"));
  Serial.print(F("Rule: ")); Serial.println(rule == LEFT_HAND ? F("LEFT-hand") : F("RIGHT-hand"));
  Serial.print(F("L_SET=")); Serial.print(L_SET, 2);
  Serial.print(F(" R_SET=")); Serial.print(R_SET, 2);
  Serial.print(F("  counts/cm=")); Serial.println(COUNTS_PER_CM, 2);
  Serial.println(F("l=line only  s=IR view  t=telemetry  c=IR calibrate  C=clear IR cal"));
  Serial.println(useCal ? F("IR: calibrated") : F("IR: fixed 800/900"));
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
  for (uint8_t i = 0; i < NUM_IR; i++) pinMode(IR_PINS[i], INPUT);

  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  lastPB = PINB;
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6);

  COUNTS_PER_CM = (ENC_PPR * 4.0 * GEAR_RATIO) / (M_PI * WHEEL_DIA_CM);
  if (ARC_RADIUS_CM > HALF_CELL) ARC_RADIUS_CM = HALF_CELL;
  resetFilters();

  EEPROM.get(CAL_ADDR, cal);
  if (cal.magic != CAL_MAGIC) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) { cal.gain[i] = 1.0; cal.offset[i] = 0.0; }
    Serial.println(F("No ultrasonic calibration in EEPROM -> raw readings."));
  } else Serial.println(F("Ultrasonic calibration loaded."));

  SidePts sp; EEPROM.get(SP_ADDR, sp);
  if (sp.magic == SP_MAGIC && sp.lSet > 2.5 && sp.lSet < 12.0 && sp.rSet > 2.5 && sp.rSet < 12.0) {
    L_SET = sp.lSet; R_SET = sp.rSet;
  } else Serial.println(F("No valid side setpoints -> run 'z' in a corridor"));

  IrCal ic; EEPROM.get(IRCAL_ADDR, ic);
  if (ic.magic == IRCAL_MAGIC) {
    for (uint8_t i = 0; i < NUM_IR; i++) { calLo[i] = ic.lo[i]; calHi[i] = ic.hi[i]; }
    useCal = true;
  }

  Serial.println(F("\n=== FINAL 2: Maze -> Line ==="));
  geometryReport();
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
  startRun();
}

void loop() {
  if (running) {
    checkAbort();
    if (!running) { stopMotors(); Serial.println(F("STOPPED")); return; }
    if (stage == ST_MAZE) mazeStep();
    else if (stage == ST_CROSS) crossStep();
    else if (millis() - lastLoopMs >= LOOP_MS) { lastLoopMs += LOOP_MS; followStep(); }
    if (abortReq) { stopMotors(); Serial.println(F("STOPPED")); }
    return;
  }

  if (!Serial.available()) return;
  char c = Serial.read();
  abortReq = false;
  switch (c) {
    case 'g': startRun(); break;
    case 'x': stopMotors(); Serial.println(F("STOPPED")); break;
    case 'd': dryRun(); break;
    case 'z': captureSetpoints(); break;
    case 'G': geometryReport(); break;
    case 'm': deadbandTest(); break;
    case 'e': { long l, r; getCounts(l, r);
                Serial.print(F("L=")); Serial.print(l); Serial.print(F(" R=")); Serial.println(r); } break;
    case 'F': Serial.println(F("Drive 1 cell")); driveCm(CELL_CM); break;
    case '<': Serial.println(F("Arc left 90"));  arcTurn(-1, arcRadius()); break;
    case '>': Serial.println(F("Arc right 90")); arcTurn(1, arcRadius());  break;
    case 'U': Serial.println(F("Spin 180"));     spinDeg(180); break;
    case 'L': rule = LEFT_HAND;  Serial.println(F("LEFT-hand rule"));  break;
    case 'R': rule = RIGHT_HAND; Serial.println(F("RIGHT-hand rule")); break;
    case 'l': startLine(true); break;
    case 's': sensorView(); break;
    case 'c': calibrate(); break;
    case 'C': clearCalibration(); break;
    case 'h': printHelp(); break;
    default: break;
  }
}