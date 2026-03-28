#include <QTRSensors.h>

// ─── Pin definitions (TB6612FNG) ────────────────────────────────────────────
#define AIN1  4
#define AIN2  5
#define PWMA  6
#define BIN1  7
#define BIN2  8
#define PWMB  9
#define STBY  3   // CRITICAL: must be HIGH or driver is disabled

// ─── Sensor pins ────────────────────────────────────────────────────────────
// Analog pins A0–A7 for 8-sensor array
const uint8_t SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const uint8_t SENSOR_COUNT   = 8;

// ─── Tunable constants ──────────────────────────────────────────────────────
#define FOLLOW_BLACK      true   // true = black line, false = white line

#define SETPOINT          3500   // Centre of 8-sensor array (range 0–7000)
#define SENSOR_THRESHOLD  200    // Calibrated value below which sensor sees nothing
#define BASE_SPEED        160    // Normal cruise speed (0–255)
#define MIN_SPEED         60     // Minimum speed during tight turns
#define RAMP_STEP         18     // Max speed change per loop tick (acceleration)
#define SEARCH_TIMEOUT    800    // ms to spin while searching for lost line
#define I_CLAMP           800.0f // Anti-windup limit for integral term

// ─── PID gains (tune these for your robot) ──────────────────────────────────
float Kp = 0.09f;
float Ki = 0.0f;
float Kd = 1.3f;

// ─── Runtime state ──────────────────────────────────────────────────────────
QTRSensors qtr;
uint16_t   sensorValues[SENSOR_COUNT];

float previousError = 0.0f;
float I             = 0.0f;

int   currentLsp = 0;   // Actual motor outputs (ramped)
int   currentRsp = 0;

unsigned long searchStart = 0;
bool          searching   = false;

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Motor driver pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Enable TB6612FNG output stage

  // Sensor init
  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);

  // ── Calibration ──────────────────────────────────────────────────────────
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibrating — sweep sensor over line for ~5 s");

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(12); // Let ADC settle between reads
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Done. Min values:");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(500);
}

// ─── Main loop ──────────────────────────────────────────────────────────────
void loop() {
  robot_control();
}

// ─── Robot control ──────────────────────────────────────────────────────────
void robot_control() {
  // Read calibrated position (0–7000 for 8 sensors)
  uint16_t position;
  if (FOLLOW_BLACK) {
    position = qtr.readLineBlack(sensorValues);
  } else {
    position = qtr.readLineWhite(sensorValues);
  }

  // ── Line loss detection ───────────────────────────────────────────────────
  bool lineLost = true;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > SENSOR_THRESHOLD) {
      lineLost = false;
      break;
    }
  }

  if (lineLost) {
    handle_line_loss();
    return;
  }

  // Line found — cancel any search
  searching = false;

  // ── PID ───────────────────────────────────────────────────────────────────
  int error = SETPOINT - (int)position;
  PID_linefollow(error);
}

// ─── PID line follow ────────────────────────────────────────────────────────
void PID_linefollow(int error) {
  float P = (float)error;

  // Reset integral when crossing the setpoint (sign change) to kill windup
  if ((error > 0) != (previousError > 0)) {
    I = 0.0f;
  }
  I = constrain(I + P, -I_CLAMP, I_CLAMP);

  float D = (float)(error - (int)previousError);

  float PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError  = (float)error;

  // Dynamic speed: slow down proportionally on sharp turns
  float turnRatio = abs(PIDvalue) / 255.0f;
  turnRatio       = constrain(turnRatio, 0.0f, 1.0f);
  int dynBase     = (int)(BASE_SPEED - turnRatio * (BASE_SPEED - MIN_SPEED));

  int targetLsp = constrain(dynBase - (int)PIDvalue, -255, 255);
  int targetRsp = constrain(dynBase + (int)PIDvalue, -255, 255);

  // Ramp toward targets (acceleration limiting)
  currentLsp = ramp(currentLsp, targetLsp, RAMP_STEP);
  currentRsp = ramp(currentRsp, targetRsp, RAMP_STEP);

  motor_drive(currentLsp, currentRsp);
}

// ─── Line loss recovery ─────────────────────────────────────────────────────
void handle_line_loss() {
  if (!searching) {
    searching   = true;
    searchStart = millis();
  }

  if (millis() - searchStart > SEARCH_TIMEOUT) {
    // Searched too long — stop and wait for manual intervention
    motor_stop();
    return;
  }

  // Spin toward the side the line was last on
  if (previousError > 0) {
    motor_drive(-180, 180); // Turn left
  } else {
    motor_drive(180, -180); // Turn right
  }
}

// ─── Acceleration ramp helper ────────────────────────────────────────────────
int ramp(int current, int target, int step) {
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return current;
}

// ─── Motor drive (TB6612FNG) ─────────────────────────────────────────────────
void motor_drive(int left, int right) {
  // Left motor — A channel
  digitalWrite(AIN1, left  >= 0 ? HIGH : LOW);
  digitalWrite(AIN2, left  >= 0 ? LOW  : HIGH);
  analogWrite(PWMA, abs(left));

  // Right motor — B channel
  digitalWrite(BIN1, right >= 0 ? HIGH : LOW);
  digitalWrite(BIN2, right >= 0 ? LOW  : HIGH);
  analogWrite(PWMB, abs(right));
}

// ─── Motor stop ──────────────────────────────────────────────────────────────
void motor_stop() {
  digitalWrite(STBY, LOW); // Disable driver — cleanest stop
  currentLsp = 0;
  currentRsp = 0;
  delay(50);
  digitalWrite(STBY, HIGH); // Re-enable for next move
}
