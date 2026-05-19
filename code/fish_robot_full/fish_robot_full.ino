// ============================================================
//  FISH ROBOT — Full Control System
//  Based on Wang et al. 2-DoF Crank-Slider Paper
//
//  Includes:
//    • Forward Kinematics (Eq. 2a–2d)
//    • Symmetric Mode   → forward swimming  (Eq. 5 / 22)
//    • Asymmetric Mode  → turning           (Eq. 6 / 23)
//    • Motor 1: Speed PID  (ω = constant assumption from paper)
//    • Motor 2: Cascaded Position → Speed PID  (robust hold against tail load)
//    • Demo sequence: fwd 3s → right 1s → fwd 1s → left 1s → repeat
// ============================================================

#include <Arduino.h>

// ─────────────────────────────────────────────
//  MECHANISM GEOMETRY  (mm / radians)
// ─────────────────────────────────────────────
const float Ra       = 15.0;   // wheel radius (mm)
const float Rb       = 20.0;   // reel radius  (mm)
const float L1       = 45.0;   // motor 1 lateral offset (mm)
const float L2       = 45.0;   // motor 2 lateral offset (mm)
const float d_wire   = 10.0;   // wire offset from spine centerline (mm)
const float L_body   = 200.0;  // elastic tail length (mm)
const float k_thrust = 0.15;   // speed model constant

// ─────────────────────────────────────────────
//  MOTOR 1 PINS  (continuously rotating — propulsion)
// ─────────────────────────────────────────────
const int M1_ENA = 9;    // PWM
const int M1_IN1 = 8;
const int M1_IN2 = 7;
const int ENC1_A = 2;    // interrupt pin
const int ENC1_B = 4;

// ─────────────────────────────────────────────
//  MOTOR 2 PINS  (cascaded position+speed — steering)
// ─────────────────────────────────────────────
const int M2_ENA = 10;   // PWM
const int M2_IN1 = 12;
const int M2_IN2 = 11;
const int ENC2_A = 3;    // interrupt pin
const int ENC2_B = 5;

// ─────────────────────────────────────────────
//  ENCODER CONSTANTS
//  JGB37-520: 20 PPR Hall effect x4 quadrature = 80 counts/rev
// ─────────────────────────────────────────────
const float PPR            = 20.0;
const float COUNTS_PER_REV = PPR * 4.0;

volatile long enc1Count = 0;
volatile long enc2Count = 0;
volatile long enc1Prev  = 0;
volatile long enc2Prev  = 0;

void enc1ISR() {
  enc1Count += (digitalRead(ENC1_A) == HIGH)
                 ? (digitalRead(ENC1_B) == LOW  ? 1 : -1)
                 : (digitalRead(ENC1_B) == HIGH ? 1 : -1);
}
void enc2ISR() {
  enc2Count += (digitalRead(ENC2_A) == HIGH)
                 ? (digitalRead(ENC2_B) == LOW  ? 1 : -1)
                 : (digitalRead(ENC2_B) == HIGH ? 1 : -1);
}

float countsToRad(long counts) {
  return (float)counts / COUNTS_PER_REV * TWO_PI;
}
float deltaToRadPerSec(long delta, float dt) {
  return ((float)delta / COUNTS_PER_REV * TWO_PI) / dt;
}

// ─────────────────────────────────────────────
//  PID STRUCT
// ─────────────────────────────────────────────
struct PID {
  float Kp, Ki, Kd;
  float integral;
  float prevError;
  float iMax;
};

int computePID(PID &pid, float error, float dt) {
  pid.integral += error * dt;
  pid.integral  = constrain(pid.integral, -pid.iMax, pid.iMax);
  float derivative = (error - pid.prevError) / dt;
  pid.prevError    = error;
  float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
  return (int)constrain(output, -255, 255);
}

void resetPID(PID &pid) {
  pid.integral  = 0.0;
  pid.prevError = 0.0;
}

// ─────────────────────────────────────────────
//  MOTOR 1 — SPEED PID
//  Maintains constant omega so kinematic model assumption holds.
//  Error = target omega - measured omega  (rad/s)
// ─────────────────────────────────────────────
//  Tuning: raise Kp until speed oscillates, halve it.
//          add Ki slowly to eliminate steady-state lag.
//          add Kd only if overshoot on speed steps.
PID pid1_spd = { 1.5, 0.8, 0.05, 0.0, 0.0, 150.0 };

// ─────────────────────────────────────────────
//  MOTOR 2 — CASCADED POSITION (outer) + SPEED (inner) PID
//
//  Outer loop: position error (rad) → commanded speed (rad/s)
//  Inner loop: speed error   (rad/s) → PWM output
//
//  Why cascaded: the elastic spine pushes back against Motor 2's
//  fixed angle phi. A position-only PID holds the angle but reacts
//  sluggishly. The inner speed loop ensures the motor actively
//  drives at whatever rate the outer loop commands, making the
//  hold stiff and the mode transition smooth.
//
//  Tuning order (critical): tune inner speed loop first to
//  stability, THEN tune outer position loop on top of it.
// ─────────────────────────────────────────────
PID pid2_pos = { 8.0, 0.5,  0.2,  0.0, 0.0, PI     };  // outer: rad → rad/s
PID pid2_spd = { 2.0, 1.0,  0.05, 0.0, 0.0, 150.0  };  // inner: rad/s → PWM

// Maximum speed the outer loop is allowed to command (rad/s)
const float M2_MAX_CMD_SPEED = TWO_PI * 6.0;

// ─────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────
const int   PID_INTERVAL_MS = 50;
const float DT              = PID_INTERVAL_MS / 1000.0;

// ─────────────────────────────────────────────
//  KINEMATICS STATE
// ─────────────────────────────────────────────
float t        = 0.0;
float th3_prev = 0.0;

// ─────────────────────────────────────────────
//  DEMO SEQUENCE
// ─────────────────────────────────────────────
const float FORWARD_OMEGA = TWO_PI * 1.0;
const float TURN_THETA2_R = -PI / 2.0;
const float TURN_THETA2_L = +PI / 2.0;
const float TURN_OMEGA    = TWO_PI * 1.0;

struct Step { uint8_t mode; unsigned long dur; };
const Step SEQUENCE[] = {
  { 0, 3000 },
  { 1, 1000 },
  { 0, 1000 },
  { 2, 1000 },
};
const int SEQ_LEN = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

int           seqIdx      = 0;
unsigned long stepStart   = 0;
uint8_t       currentMode = 0;

// ─────────────────────────────────────────────
//  FORWARD KINEMATICS  (Eq. 2a-2d)
// ─────────────────────────────────────────────
bool forwardKinematics(float th1, float th2, float &th3) {
  float A = Ra * Rb * (cos(th1) - cos(th2)) + Rb * (L1 + L2);
  float B = Ra * Rb * (sin(th2) - sin(th1));
  float C = Ra * Ra * sin(th1 - th2) - Ra * (sin(th1) * L2 + sin(th2) * L1);
  float R = sqrt(A * A + B * B);
  if (abs(C / R) > 1.0) return false;
  th3 = asin(C / R) - atan2(B, A);
  return true;
}

// ─────────────────────────────────────────────
//  TAIL TIP POSITION  (Eqs. 9-10)
// ─────────────────────────────────────────────
void tailTipPosition(float theta_a, float &tipX, float &tipY) {
  if (abs(theta_a) < 0.001) {
    tipX = L_body; tipY = 0.0;
  } else {
    float r = L_body / theta_a;
    tipX = r * sin(theta_a);
    tipY = r * (1.0 - cos(theta_a));
  }
}

// ─────────────────────────────────────────────
//  SWIMMING SPEED ESTIMATE  (Eqs. 14-15)
// ─────────────────────────────────────────────
float estimateSpeed(float theta_a, float theta_a_dot) {
  float tip_speed;
  if (abs(theta_a) < 0.001) {
    tip_speed = L_body * abs(theta_a_dot);
  } else {
    float dY = (L_body / (theta_a * theta_a)) * (sin(theta_a) - theta_a * cos(theta_a));
    tip_speed = abs(dY * theta_a_dot);
  }
  return k_thrust * tip_speed;
}

// ─────────────────────────────────────────────
//  SYMMETRIC MODE  (Eq. 5 / 22)
//  theta1 = theta2 = omega*t → 1-DoF, max thrust
//  Returns: target omega for both motors (rad/s)
// ─────────────────────────────────────────────
float symmetricMode(float omega, float &th3, float &th3_dot, float &v_robot) {
  float th1 = omega * t;
  float th2 = th1;
  if (!forwardKinematics(th1, th2, th3)) {
    Serial.println("[SYM] FK unreachable");
    return omega;
  }
  th3_dot = (th3 - th3_prev) / DT;
  th3_prev = th3;
  float theta_a     = (Rb / d_wire) * th3;
  float theta_a_dot = (Rb / d_wire) * th3_dot;
  v_robot = estimateSpeed(theta_a, theta_a_dot);
  return omega;
}

// ─────────────────────────────────────────────
//  ASYMMETRIC MODE  (Eq. 6 / 23)
//  Motor 1: rotates at omega (propulsion)
//  Motor 2: held at phi by cascaded PID (steering)
//  Returns: target omega for Motor 1 (rad/s)
// ─────────────────────────────────────────────
float asymmetricMode(float omega, float phi,
                     float &th3, float &th3_dot, float &v_robot) {
  float th1 = omega * t;
  float th2 = phi;
  if (!forwardKinematics(th1, th2, th3)) {
    Serial.println("[ASYM] FK unreachable");
    return omega;
  }
  th3_dot = (th3 - th3_prev) / DT;
  th3_prev = th3;
  float theta_a     = (Rb / d_wire) * th3;
  float theta_a_dot = (Rb / d_wire) * th3_dot;
  v_robot = estimateSpeed(theta_a, theta_a_dot);
  return omega;
}

// ─────────────────────────────────────────────
//  LOW-LEVEL MOTOR DRIVE
// ─────────────────────────────────────────────
void driveMotor1(int pwm) {
  if (pwm >= 0) { digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);  }
  else          { digitalWrite(M1_IN1, LOW);  digitalWrite(M1_IN2, HIGH); pwm = -pwm; }
  analogWrite(M1_ENA, constrain(pwm, 0, 255));
}
void driveMotor2(int pwm) {
  if (pwm >= 0) { digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);  }
  else          { digitalWrite(M2_IN1, LOW);  digitalWrite(M2_IN2, HIGH); pwm = -pwm; }
  analogWrite(M2_ENA, constrain(pwm, 0, 255));
}

// ─────────────────────────────────────────────
//  SEQUENCE MANAGER
// ─────────────────────────────────────────────
uint8_t updateSequence() {
  if (millis() - stepStart >= SEQUENCE[seqIdx].dur) {
    seqIdx    = (seqIdx + 1) % SEQ_LEN;
    stepStart = millis();
    resetPID(pid1_spd);
    resetPID(pid2_pos);
    resetPID(pid2_spd);
    Serial.print("[SEQ] Step → "); Serial.println(seqIdx);
  }
  return SEQUENCE[seqIdx].mode;
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("=== Fish Robot — Full Control System ===");

  pinMode(M1_ENA, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_ENA, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);

  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, CHANGE);

  stepStart = millis();
  Serial.println("mode | m1_tgt w | m1_act w | m2_tgt deg | m2_act deg | th3 | theta_a | v_robot");
}

// ─────────────────────────────────────────────
//  MAIN LOOP  (20 Hz)
// ─────────────────────────────────────────────
static unsigned long lastPID = 0;

void loop() {
  unsigned long now = millis();
  if (now - lastPID < PID_INTERVAL_MS) return;
  lastPID = now;

  t = now / 1000.0;
  currentMode = updateSequence();

  // ── Read encoders atomically ──
  noInterrupts();
  long c1 = enc1Count; long c1p = enc1Prev; enc1Prev = c1;
  long c2 = enc2Count; long c2p = enc2Prev; enc2Prev = c2;
  interrupts();

  // Measured velocities (rad/s) from delta counts over DT
  float m1_omega_act = deltaToRadPerSec(c1 - c1p, DT);
  float m2_omega_act = deltaToRadPerSec(c2 - c2p, DT);

  // Measured positions (rad)
  float m2_pos_act = countsToRad(c2);

  // ── Kinematics ──
  float th3 = 0, th3_dot = 0, v_robot = 0;
  float m1_target_omega = 0.0;
  float m2_target_pos   = 0.0;

  if (currentMode == 0) {
    // Symmetric: Motor 1 and Motor 2 both spin at omega
    // Motor 2 position tracks Motor 1's live position
    m1_target_omega = symmetricMode(FORWARD_OMEGA, th3, th3_dot, v_robot);
    m2_target_pos   = countsToRad(c1);   // mirror Motor 1

  } else {
    float phi = (currentMode == 1) ? TURN_THETA2_R : TURN_THETA2_L;
    m1_target_omega = asymmetricMode(TURN_OMEGA, phi, th3, th3_dot, v_robot);
    m2_target_pos   = phi;
  }

  // ══════════════════════════════════════════
  //  MOTOR 1 — SPEED PID
  //  Ensures omega stays constant as the paper assumes.
  // ══════════════════════════════════════════
  float err1 = m1_target_omega - m1_omega_act;
  int pwm1 = computePID(pid1_spd, err1, DT);
  driveMotor1(pwm1);

  // ══════════════════════════════════════════
  //  MOTOR 2 — CASCADED PID
  //
  //  OUTER (position): error_pos (rad) → commanded speed (rad/s)
  //  INNER (speed):    error_spd (rad/s) → PWM
  //
  //  The outer loop tells Motor 2 how fast to move.
  //  The inner loop makes it actually move at that speed
  //  regardless of the elastic tail pushing back.
  // ══════════════════════════════════════════

  // Wrap position error to [-pi, pi]
  float err2_pos = m2_target_pos - m2_pos_act;
  while (err2_pos >  PI) err2_pos -= TWO_PI;
  while (err2_pos < -PI) err2_pos += TWO_PI;

  // Outer loop output = commanded speed, clamped
  float m2_cmd_spd = (float)computePID(pid2_pos, err2_pos, DT);
  m2_cmd_spd = constrain(m2_cmd_spd, -M2_MAX_CMD_SPEED, M2_MAX_CMD_SPEED);

  // Inner loop: speed error → PWM
  float err2_spd = m2_cmd_spd - m2_omega_act;
  int pwm2 = computePID(pid2_spd, err2_spd, DT);
  driveMotor2(pwm2);

  // ── Serial telemetry ──
  float theta_a = (Rb / d_wire) * th3;
  Serial.print(currentMode);                     Serial.print(" | ");
  Serial.print(m1_target_omega, 2);              Serial.print(" | ");
  Serial.print(m1_omega_act, 2);                 Serial.print(" | ");
  Serial.print(m2_target_pos * 180.0 / PI, 1);  Serial.print("deg | ");
  Serial.print(m2_pos_act    * 180.0 / PI, 1);  Serial.print("deg | ");
  Serial.print(th3, 3);                          Serial.print(" | ");
  Serial.print(theta_a, 3);                      Serial.print(" | ");
  Serial.print(v_robot, 2);                      Serial.println(" mm/s");
}
