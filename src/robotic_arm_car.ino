/*
 * ============================================================
 *  Mobile Robotic Arm Car — Pick & Place via Bluetooth
 *  Platform : Arduino Uno / Nano
 *  Chassis  : 4-wheel DC motor car (L298N driver)
 *  Arm      : 4-DOF servo arm + gripper
 *  Control  : HC-05 / HC-06 Bluetooth module (Serial)
 * ============================================================
 *
 *  --- PIN MAP ---
 *
 *  L298N Motor Driver:
 *    ENA  → D5  (PWM speed Left)
 *    IN1  → D4
 *    IN2  → D3
 *    ENB  → D6  (PWM speed Right)
 *    IN3  → D7
 *    IN4  → D8
 *
 *  Servo Motors (via PWM pins):
 *    BASE_SERVO    → D9   (waist rotation)
 *    SHOULDER_SERVO→ D10  (upper arm lift)
 *    ELBOW_SERVO   → D11  (forearm)
 *    GRIPPER_SERVO → D12  (claw open/close)
 *
 *  Bluetooth HC-05:
 *    TX → D0 (Arduino RX)
 *    RX → D1 (Arduino TX) via 1kΩ/2kΩ divider
 *
 * ============================================================
 *  BLUETOOTH COMMAND SET (send single character from app)
 *
 *  CHASSIS:
 *    'F' – Forward          'B' – Backward
 *    'L' – Turn Left        'R' – Turn Right
 *    'S' – Stop
 *
 *  ARM JOINTS (each press = 5° step):
 *    'q' – Base CW          'a' – Base CCW
 *    'w' – Shoulder Up      's' – Shoulder Down
 *    'e' – Elbow Up         'd' – Elbow Down
 *    'o' – Gripper Open     'c' – Gripper Close
 *
 *  AUTO SEQUENCE:
 *    'P' – Execute full Pick & Place sequence
 *    'H' – Home position
 * ============================================================
 */

#include <Servo.h>

// ── Motor Driver Pins ────────────────────────────────────────
#define ENA  5
#define IN1  4
#define IN2  3
#define ENB  6
#define IN3  7
#define IN4  8

// ── Servo Pins ───────────────────────────────────────────────
#define BASE_PIN      9
#define SHOULDER_PIN  10
#define ELBOW_PIN     11
#define GRIPPER_PIN   12

// ── Speed & Step Settings ────────────────────────────────────
#define MOTOR_SPEED   180   // 0–255
#define SERVO_STEP    5     // degrees per button press

// ── Servo Angle Limits ───────────────────────────────────────
#define BASE_MIN       0
#define BASE_MAX     180
#define SHOULDER_MIN  30
#define SHOULDER_MAX 150
#define ELBOW_MIN     0
#define ELBOW_MAX   150
#define GRIPPER_OPEN  70
#define GRIPPER_CLOSE 10

// ── Home Positions ───────────────────────────────────────────
#define BASE_HOME      90
#define SHOULDER_HOME  90
#define ELBOW_HOME     90
#define GRIPPER_HOME   GRIPPER_OPEN

Servo baseServo, shoulderServo, elbowServo, gripperServo;

int baseAngle     = BASE_HOME;
int shoulderAngle = SHOULDER_HOME;
int elbowAngle    = ELBOW_HOME;
int gripperAngle  = GRIPPER_HOME;

// ── Helper: clamp value ──────────────────────────────────────
int clamp(int val, int minVal, int maxVal) {
  return max(minVal, min(maxVal, val));
}

// ── Smooth servo move ────────────────────────────────────────
void smoothMove(Servo &srv, int &current, int target, int delayMs = 15) {
  int step = (target > current) ? 1 : -1;
  while (current != target) {
    current += step;
    srv.write(current);
    delay(delayMs);
  }
}

// ── Chassis control ──────────────────────────────────────────
void driveForward() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void driveBackward() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}
void turnLeft() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void turnRight() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}
void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ── Home all servos ──────────────────────────────────────────
void goHome() {
  smoothMove(gripperServo, gripperAngle, GRIPPER_HOME);
  smoothMove(elbowServo,   elbowAngle,   ELBOW_HOME);
  smoothMove(shoulderServo,shoulderAngle,SHOULDER_HOME);
  smoothMove(baseServo,    baseAngle,    BASE_HOME);
}

// ── Auto Pick & Place Sequence ───────────────────────────────
/*
 *  Forward Kinematics reference:
 *    L1 = upper arm length (shoulder→elbow)  ≈ 100 mm
 *    L2 = forearm length   (elbow→gripper)   ≈  90 mm
 *
 *  For a target at distance r from base, height h:
 *    cos(θ₂) = (r²+h²-L1²-L2²) / (2·L1·L2)
 *    θ₁ = atan2(h,r) - atan2(L2·sin(θ₂), L1+L2·cos(θ₂))
 *
 *  Angles below are pre-computed for a demo pick at ~12 cm
 *  in front of the base, ground level.
 */
void pickAndPlace() {
  // 1. Open gripper & raise arm to safe travel
  smoothMove(gripperServo,  gripperAngle, GRIPPER_OPEN);
  smoothMove(shoulderServo, shoulderAngle, 100);
  smoothMove(elbowServo,    elbowAngle,   60);
  delay(300);

  // 2. Rotate base to pick position (front)
  smoothMove(baseServo, baseAngle, 90);
  delay(200);

  // 3. Lower arm to object level
  smoothMove(shoulderServo, shoulderAngle, 60);
  smoothMove(elbowServo,    elbowAngle,   120);
  delay(400);

  // 4. Close gripper — PICK
  smoothMove(gripperServo, gripperAngle, GRIPPER_CLOSE);
  delay(400);

  // 5. Lift object
  smoothMove(shoulderServo, shoulderAngle, 100);
  smoothMove(elbowServo,    elbowAngle,   60);
  delay(300);

  // 6. Rotate base to place position (side, 45°)
  smoothMove(baseServo, baseAngle, 45);
  delay(300);

  // 7. Lower to place level
  smoothMove(shoulderServo, shoulderAngle, 65);
  smoothMove(elbowServo,    elbowAngle,   115);
  delay(400);

  // 8. Open gripper — PLACE
  smoothMove(gripperServo, gripperAngle, GRIPPER_OPEN);
  delay(300);

  // 9. Return home
  goHome();
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  // Motor driver pins
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  // Servo attach
  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);

  // Move to home
  baseServo.write(BASE_HOME);
  shoulderServo.write(SHOULDER_HOME);
  elbowServo.write(ELBOW_HOME);
  gripperServo.write(GRIPPER_HOME);

  Serial.begin(9600);   // HC-05 default baud rate
  delay(500);
}

// ── Main Loop ────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      // ── CHASSIS ──
      case 'F': driveForward();  break;
      case 'B': driveBackward(); break;
      case 'L': turnLeft();      break;
      case 'R': turnRight();     break;
      case 'S': stopMotors();    break;

      // ── BASE ──
      case 'q':
        baseAngle = clamp(baseAngle + SERVO_STEP, BASE_MIN, BASE_MAX);
        baseServo.write(baseAngle); break;
      case 'a':
        baseAngle = clamp(baseAngle - SERVO_STEP, BASE_MIN, BASE_MAX);
        baseServo.write(baseAngle); break;

      // ── SHOULDER ──
      case 'w':
        shoulderAngle = clamp(shoulderAngle + SERVO_STEP, SHOULDER_MIN, SHOULDER_MAX);
        shoulderServo.write(shoulderAngle); break;
      case 's':
        shoulderAngle = clamp(shoulderAngle - SERVO_STEP, SHOULDER_MIN, SHOULDER_MAX);
        shoulderServo.write(shoulderAngle); break;

      // ── ELBOW ──
      case 'e':
        elbowAngle = clamp(elbowAngle + SERVO_STEP, ELBOW_MIN, ELBOW_MAX);
        elbowServo.write(elbowAngle); break;
      case 'd':
        elbowAngle = clamp(elbowAngle - SERVO_STEP, ELBOW_MIN, ELBOW_MAX);
        elbowServo.write(elbowAngle); break;

      // ── GRIPPER ──
      case 'o':
        gripperAngle = clamp(gripperAngle + SERVO_STEP, GRIPPER_CLOSE, GRIPPER_OPEN);
        gripperServo.write(gripperAngle); break;
      case 'c':
        gripperAngle = clamp(gripperAngle - SERVO_STEP, GRIPPER_CLOSE, GRIPPER_OPEN);
        gripperServo.write(gripperAngle); break;

      // ── AUTO SEQUENCES ──
      case 'P': pickAndPlace(); break;
      case 'H': goHome();       break;

      default: break;
    }
  }
}
