/*
 * 3x Ultrasonic Sensor Decision Test - Arduino Mega
 * ---------------------------------------------------
 * Pins:
 *   Front : TRIG 38, ECHO 39
 *   Left  : TRIG 36, ECHO 37
 *   Right : TRIG 40, ECHO 41
 *
 * Decision logic (front has first priority):
 *   1. Front clear              -> FORWARD
 *   2. Front blocked:
 *        Left & Right clear     -> turn toward the more open side
 *        Only Left clear        -> LEFT
 *        Only Right clear       -> RIGHT
 *        All blocked            -> U-TURN
 *
 * No motors are driven - decisions are printed to Serial Monitor (115200 baud).
 */

// ---------------- Pin configuration ----------------
const uint8_t FRONT_TRIG = 38, FRONT_ECHO = 39;
const uint8_t LEFT_TRIG  = 36, LEFT_ECHO  = 37;
const uint8_t RIGHT_TRIG = 40, RIGHT_ECHO = 41;

// ---------------- Tunable thresholds (cm) ----------------
const float FRONT_THRESHOLD = 15.0;   // obstacle ahead if closer than this
const float SIDE_THRESHOLD  = 15.0;   // side counts as "open" if farther than this
const float MAX_DISTANCE    = 300.0;  // reported when no echo (out of range)

// pulseIn timeout: ~25 ms covers ~4 m round trip
const unsigned long ECHO_TIMEOUT_US = 25000UL;

// Delay between sensor pings to avoid cross-talk between sensors
const unsigned int INTER_SENSOR_DELAY_MS = 30;

enum Decision { FORWARD, TURN_LEFT, TURN_RIGHT, U_TURN };

// ---------------- Distance measurement ----------------
float readDistanceOnce(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return MAX_DISTANCE;        // no echo -> treat as open
  return duration * 0.0343 / 2.0;                // speed of sound 343 m/s
}

// Median of 3 readings to reject spikes
float readDistance(uint8_t trigPin, uint8_t echoPin) {
  float a = readDistanceOnce(trigPin, echoPin); delay(10);
  float b = readDistanceOnce(trigPin, echoPin); delay(10);
  float c = readDistanceOnce(trigPin, echoPin);

  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

// ---------------- Decision logic ----------------
Decision decide(float front, float left, float right) {
  // Priority 1: front sensor
  if (front > FRONT_THRESHOLD) return FORWARD;

  // Front blocked -> check sides
  bool leftOpen  = left  > SIDE_THRESHOLD;
  bool rightOpen = right > SIDE_THRESHOLD;

  if (leftOpen && rightOpen) return (left >= right) ? TURN_LEFT : TURN_RIGHT;
  if (leftOpen)              return TURN_LEFT;
  if (rightOpen)             return TURN_RIGHT;
  return U_TURN;
}

const char* decisionName(Decision d) {
  switch (d) {
    case FORWARD:    return "FORWARD";
    case TURN_LEFT:  return "TURN LEFT";
    case TURN_RIGHT: return "TURN RIGHT";
    case U_TURN:     return "U-TURN";
  }
  return "UNKNOWN";
}

// ---------------- Setup & loop ----------------
void setup() {
  Serial.begin(115200);

  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG,  OUTPUT); pinMode(LEFT_ECHO,  INPUT);
  pinMode(RIGHT_TRIG, OUTPUT); pinMode(RIGHT_ECHO, INPUT);

  digitalWrite(FRONT_TRIG, LOW);
  digitalWrite(LEFT_TRIG,  LOW);
  digitalWrite(RIGHT_TRIG, LOW);

  Serial.println(F("Ultrasonic decision test started"));
  Serial.println(F("Front\tLeft\tRight\t-> Decision"));
}

void loop() {
  float front = readDistance(FRONT_TRIG, FRONT_ECHO);
  delay(INTER_SENSOR_DELAY_MS);
  float left  = readDistance(LEFT_TRIG, LEFT_ECHO);
  delay(INTER_SENSOR_DELAY_MS);
  float right = readDistance(RIGHT_TRIG, RIGHT_ECHO);

  Decision d = decide(front, left, right);

  Serial.print(front, 1); Serial.print(F("\t"));
  Serial.print(left, 1);  Serial.print(F("\t"));
  Serial.print(right, 1); Serial.print(F("\t-> "));
  Serial.println(decisionName(d));

  delay(200);
}
