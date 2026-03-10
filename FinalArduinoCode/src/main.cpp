#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>

// Base (CL57T #1)
const int STEP_PIN_B = 24;
const int DIR_PIN_B  = 25;
const int ALM_PIN_B  = 32;
const long GEAR_RATIO_B = 10;

// Elbow (CL57T #2)
const int STEP_PIN_E = 22;
const int DIR_PIN_E  = 23;
const int ALM_PIN_E  = 33;
const long GEAR_RATIO_E = 100;

// Wrist move (CL57T #3)
const int STEP_PIN_W = 26;
const int DIR_PIN_W  = 27;
const int ALM_PIN_W  = 34;
const long GEAR_RATIO_W = 10;

// Shoulder (CL86T)
const int STEP_PIN_S = 30;
const int DIR_PIN_S  = 31;
const int ALM_PIN_S  = 36;
const long GEAR_RATIO_S = 100;

// Homing Pins
const int HOME_PIN_B = -1;
const int HOME_PIN_S = -1;
const int HOME_PIN_E = -1;
const int HOME_PIN_W = -1;

const long PULSES_PER_MOTOR_REV = 6400;
const long PULSES_PER_OUTPUT_REV_B = PULSES_PER_MOTOR_REV * GEAR_RATIO_B;
const long PULSES_PER_OUTPUT_REV_S = PULSES_PER_MOTOR_REV * GEAR_RATIO_S;
const long PULSES_PER_OUTPUT_REV_E = PULSES_PER_MOTOR_REV * GEAR_RATIO_E;
const long PULSES_PER_OUTPUT_REV_W = PULSES_PER_MOTOR_REV * GEAR_RATIO_W;

const float MAX_B = 1000.0f, ACC_B = 500.0f;
const float MAX_W = 1000.0f, ACC_W = 500.0f;
const float MAX_S = 8000.0f, ACC_S = 4000.0f;
const float MAX_E = 8000.0f, ACC_E = 4000.0f;

AccelStepper base(AccelStepper::DRIVER, STEP_PIN_B, DIR_PIN_B);
AccelStepper shoulder(AccelStepper::DRIVER, STEP_PIN_S, DIR_PIN_S);
AccelStepper elbow(AccelStepper::DRIVER, STEP_PIN_E, DIR_PIN_E);
AccelStepper wrist(AccelStepper::DRIVER, STEP_PIN_W, DIR_PIN_W);

const float SIGN_B =  1.0f;
const float SIGN_S = -1.0f;   // flip shoulder
const float SIGN_E = 1.0f;
const float SIGN_W = -1.0f;   // flip wrist

const float HOME_ANGLE = 90.0f;

long degToSteps(float deg, long pulsesPerOutRev) {
  return lround((deg / 360.0f) * (float)pulsesPerOutRev);
}

bool anyAlarmActive() {
  return (digitalRead(ALM_PIN_B) == LOW) ||
         (digitalRead(ALM_PIN_E) == LOW) ||
         (digitalRead(ALM_PIN_W) == LOW) ||
         (digitalRead(ALM_PIN_S) == LOW);
}

void runAllUntilDone() {
  while (base.distanceToGo() || elbow.distanceToGo() || wrist.distanceToGo() || shoulder.distanceToGo()) {
    base.run();
    elbow.run();
    wrist.run();
    shoulder.run();

    if (anyAlarmActive()) {
      Serial.println("ALARM ACTIVE during move — stopping all.");
      base.stop();
      elbow.stop();
      wrist.stop();
      shoulder.stop();

      while (base.isRunning() || elbow.isRunning() || wrist.isRunning() || shoulder.isRunning()) {
        base.run();
        elbow.run();
        wrist.run();
        shoulder.run();
      }
      return;
    }
  }
}

void moveToMappedPos(float baseDeg, float shoulderDeg, float elbowDeg, float wristDeg) {
  long baseTarget     = degToSteps(SIGN_B * baseDeg,     PULSES_PER_OUTPUT_REV_B);
  long shoulderTarget = degToSteps(SIGN_S * shoulderDeg, PULSES_PER_OUTPUT_REV_S);
  long elbowTarget    = degToSteps(SIGN_E * elbowDeg,    PULSES_PER_OUTPUT_REV_E);
  long wristTarget    = degToSteps(SIGN_W * wristDeg,    PULSES_PER_OUTPUT_REV_W);

  base.moveTo(baseTarget);
  shoulder.moveTo(shoulderTarget);
  elbow.moveTo(elbowTarget);
  wrist.moveTo(wristTarget);

  runAllUntilDone();
}

void moveByDeg(float baseDeg, float shoulderDeg, float elbowDeg, float wristDeg) {
  long baseSteps     = degToSteps(SIGN_B * baseDeg,     PULSES_PER_OUTPUT_REV_B);
  long shoulderSteps = degToSteps(SIGN_S * shoulderDeg, PULSES_PER_OUTPUT_REV_S);
  long elbowSteps    = degToSteps(SIGN_E * elbowDeg,    PULSES_PER_OUTPUT_REV_E);
  long wristSteps    = degToSteps(SIGN_W * wristDeg,    PULSES_PER_OUTPUT_REV_W);

  base.move(baseSteps);
  shoulder.move(shoulderSteps);
  elbow.move(elbowSteps);
  wrist.move(wristSteps);

  runAllUntilDone();
}

void homeJoint(AccelStepper &motor, int homePin, long pulsesPerOutRev, float homingSpeed) {
  motor.setMaxSpeed(homingSpeed);
  motor.setAcceleration(homingSpeed / 2.0f);

  motor.move(degToSteps(360.0f, pulsesPerOutRev));

  while (motor.distanceToGo() > 0) {
    motor.run();
    if (digitalRead(homePin) == LOW) {  // switch triggered
      motor.stop();
      while (motor.isRunning()) motor.run();  // decelerate to stop
      motor.setCurrentPosition(degToSteps(HOME_ANGLE, pulsesPerOutRev));
      Serial.println("Switch hit.");
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(ALM_PIN_B, INPUT_PULLUP);
  pinMode(ALM_PIN_S, INPUT_PULLUP);
  pinMode(ALM_PIN_E, INPUT_PULLUP);
  pinMode(ALM_PIN_W, INPUT_PULLUP);

  // pinMode(HOME_PIN_B, INPUT_PULLUP);
  // pinMode(HOME_PIN_S, INPUT_PULLUP);
  // pinMode(HOME_PIN_E, INPUT_PULLUP);
  // pinMode(HOME_PIN_W, INPUT_PULLUP);

  // Apply per-joint speed/accel
  base.setMaxSpeed(MAX_B);
  base.setAcceleration(ACC_B);

  wrist.setMaxSpeed(MAX_W);
  wrist.setAcceleration(ACC_W);

  shoulder.setMaxSpeed(MAX_S);
  shoulder.setAcceleration(ACC_S);

  elbow.setMaxSpeed(MAX_E);
  elbow.setAcceleration(ACC_E);

  if (anyAlarmActive()) {
    Serial.println("ALARM ACTIVE at startup — clear faults first.");
    return;
  }
}

void loop() {
  if (anyAlarmActive()) {
    Serial.println("ALARM ACTIVE in loop — clear faults first.");
    while (true) delay(1000);
  }

  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    Serial.println("Received: " + msg);

    if (msg == "home") {
      homeJoint(base, HOME_PIN_B, PULSES_PER_OUTPUT_REV_B, MAX_B / 2.0f);
      homeJoint(shoulder, HOME_PIN_S, PULSES_PER_OUTPUT_REV_S, MAX_S / 2.0f);
      homeJoint(elbow, HOME_PIN_E, PULSES_PER_OUTPUT_REV_E, MAX_E / 2.0f);
      homeJoint(wrist, HOME_PIN_W, PULSES_PER_OUTPUT_REV_W, MAX_W / 2.0f);
      base.setMaxSpeed(MAX_B); base.setAcceleration(ACC_B);
      shoulder.setMaxSpeed(MAX_S); shoulder.setAcceleration(ACC_S);
      elbow.setMaxSpeed(MAX_E); elbow.setAcceleration(ACC_E);
      wrist.setMaxSpeed(MAX_W); wrist.setAcceleration(ACC_W);
      moveToMappedPos(0.0f, 0.0f, 90.0f, 180.0f);
    }
    else {
      char angles[msg.length() + 1];
      msg.toCharArray(angles, sizeof(angles));

      char *token = strtok(angles, ",");
      float base_deg    = atof(token);
      token = strtok(NULL, ",");
      float shoulder_deg = atof(token);
      token = strtok(NULL, ",");
      float elbow_deg   = atof(token);
      token = strtok(NULL, ",");
      float wrist_deg   = atof(token);

      // moveToMappedPos(base_deg, shoulder_deg, elbow_deg, wrist_deg);
      moveByDeg(base_deg, shoulder_deg, elbow_deg, wrist_deg);
      Serial.println("OK");
    }
  }
}