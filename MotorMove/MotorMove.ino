#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

float stepsPerDeg_J1 = (42000.0 * 100.0) / 360.0; // Shoulder
float stepsPerDeg_J2 = (42000.0 * 100.0) / 360.0; // Elbow
float stepsPerDeg_J3 = (42000.0 * 13.0)  / 360.0; // Wrist

const int PIN_STEP1 = 2;  // wrist
const int PIN_DIR1  = 3;
const int PIN_BRK1  = 4;
const int PIN_ALM1  = 5;

const int PIN_STEP2 = 6;  // elbow
const int PIN_DIR2  = 7;
const int PIN_BRK2  = 8;
const int PIN_ALM2  = 9;

const int PIN_STEP3 = 10; // shoulder
const int PIN_DIR3  = 11;
const int PIN_BRK3  = 12;
const int PIN_ALM3  = 13;

// Initializing all motors
AccelStepper stepJ1(AccelStepper::DRIVER, PIN_STEP1, PIN_DIR1); // shoulder
AccelStepper stepJ2(AccelStepper::DRIVER, PIN_STEP2, PIN_DIR2); // elbow
AccelStepper stepJ3(AccelStepper::DRIVER, PIN_STEP3, PIN_DIR3); // wrist
MultiStepper steppers;

float MAX_SPEED = 1000;

void moveToAngles(float j1, float j2, float j3) {
  long targets[3];
  targets[0] = lround(j1 * stepsPerDeg_J1);
  targets[1] = lround(j2 * stepsPerDeg_J2);
  targets[2] = lround(j3 * stepsPerDeg_J3);

  steppers.moveTo(targets);

  while (stepJ1.distanceToGo() || stepJ2.distanceToGo() || stepJ3.distanceToGo()) {
    stepJ1.run();
    stepJ2.run();
    stepJ3.run();
  }
}

void getCurrentAngles(float &j1, float &j2, float &j3) {
  j1 = stepJ1.currentPosition() / stepsPerDeg_J1;
  j2 = stepJ2.currentPosition() / stepsPerDeg_J2;
  j3 = stepJ3.currentPosition() / stepsPerDeg_J3;
}

void testOneJointRelative(const char* name, int jointIndex, float deltaDeg, int dwellMs = 800) {
  Serial.print("\n--- Testing "); Serial.print(name); Serial.println(" (relative) ---");

  float j1, j2, j3;
  getCurrentAngles(j1, j2, j3);

  float j1_start = j1, j2_start = j2, j3_start = j3;

  // +delta
  if (jointIndex == 1) j1 += deltaDeg;
  if (jointIndex == 2) j2 += deltaDeg;
  if (jointIndex == 3) j3 += deltaDeg;
  moveToAngles(j1, j2, j3);
  delay(dwellMs);

  // back
  moveToAngles(j1_start, j2_start, j3_start);
  delay(dwellMs);

  // -delta
  j1 = j1_start; j2 = j2_start; j3 = j3_start;
  if (jointIndex == 1) j1 -= deltaDeg;
  if (jointIndex == 2) j2 -= deltaDeg;
  if (jointIndex == 3) j3 -= deltaDeg;
  moveToAngles(j1, j2, j3);
  delay(dwellMs);

  // back
  moveToAngles(j1_start, j2_start, j3_start);
  delay(dwellMs);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  stepJ1.setMaxSpeed(MAX_SPEED);
  stepJ2.setMaxSpeed(MAX_SPEED);
  stepJ3.setMaxSpeed(MAX_SPEED);

  steppers.addStepper(stepJ1);
  steppers.addStepper(stepJ2);
  steppers.addStepper(stepJ3);

  // Relative test: define "start" as zero for all joints
  stepJ1.setCurrentPosition(0);
  stepJ2.setCurrentPosition(0);
  stepJ3.setCurrentPosition(0);

  const float DELTA = 30.0;
  testOneJointRelative("Shoulder (J1)", 1, DELTA);
  testOneJointRelative("Elbow (J2)",    2, DELTA);
  testOneJointRelative("Wrist (J3)",    3, DELTA);
}

void loop() {}