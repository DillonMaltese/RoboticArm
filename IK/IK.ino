#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

float L1 = 34.0;   // Shoulder-to-elbow [in]
float L2 = 22.0;   // Elbow-to-wrist [in]
float L3 = 12.0;   // Wrist-to-probe tip [in]
float move = 3.0; // distance to move forward (+x) [in]

// Steps per degree for each joint (motor steps * microsteps * gearbox / 360)
float stepsPerDeg_J1 = (200.0 * 42000.0 * 100.0) / 360.0; // Shoulder
float stepsPerDeg_J2 = (200.0 * 42000.0 * 100.0) / 360.0; // Elbow
float stepsPerDeg_J3 = (200.0 * 42000.0 * 13.0)  / 360.0; // Wrist

// Starting joint angles (deg)
float shoulder_deg = 30.0;
float elbow_deg    = 45.0;
float wrist_deg    = -90.0 - (shoulder_deg + elbow_deg); // keep probe vertical

// Pins
#define J1_STEP 10 //Shoulder
#define J1_DIR  11
#define J1_BRK  12
#define J1_ALM  13
#define J2_STEP 6  //Elbow
#define J2_DIR  7
#define J2_BRK  8
#define J2_ALM  9
#define J3_STEP 2  //Wrist
#define J3_DIR  3
#define J3_BRK  4
#define J3_ALM  5

AccelStepper stepJ1(AccelStepper::DRIVER, J1_STEP, J1_DIR);
AccelStepper stepJ2(AccelStepper::DRIVER, J2_STEP, J2_DIR);
AccelStepper stepJ3(AccelStepper::DRIVER, J3_STEP, J3_DIR);
MultiStepper steppers;

float MAX_SPEED = 2000;
float ACCEL     = 4000;

void forwardKinematics(float j1, float j2, float &x, float &z) {
  float t1 = radians(j1);
  float t2 = radians(j2);
  // wrist center
  float xw = L1*cos(t1) + L2*cos(t1 + t2);
  float zw = L1*sin(t1) + L2*sin(t1 + t2);
  // tool tip (probe) offset by L3 vertically downward (γ = -90°)
  x = xw + L3*cos(radians(-90));
  z = zw + L3*sin(radians(-90));
}

bool inverseKinematics(float x, float z, float &j1_out, float &j2_out, float &j3_out) {
  // Wrist center position (since γ=-90°, the probe points down)
  float xw = x - L3*cos(radians(-90)); // same as x
  float zw = z - L3*sin(radians(-90)); // z + L3

  float D = (sq(xw) + sq(zw) - sq(L1) - sq(L2)) / (2*L1*L2);
  if (D > 1 || D < -1) return false;

  float th2 = atan2(sqrt(1 - D*D), D); // elbow-up
  float th1 = atan2(zw, xw) - atan2(L2*sin(th2), L1 + L2*cos(th2));
  float th3 = radians(-90) - (th1 + th2); // wrist keeps probe vertical

  j1_out = degrees(th1);
  j2_out = degrees(th2);
  j3_out = degrees(th3);
  return true;
}

void moveToAngles(float j1, float j2, float j3) {
  long targets[3];
  targets[0] = lround(j1 * stepsPerDeg_J1);
  targets[1] = lround(j2 * stepsPerDeg_J2);
  targets[2] = lround(j3 * stepsPerDeg_J3);
  steppers.moveTo(targets);

  bool running = true;
  while (running) {
    stepJ1.run();
    stepJ2.run();
    stepJ3.run();
    running = (stepJ1.distanceToGo() || stepJ2.distanceToGo() || stepJ3.distanceToGo());
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("3-Joint IK Demo");

  // Setup steppers
  stepJ1.setMaxSpeed(MAX_SPEED); stepJ1.setAcceleration(ACCEL);
  stepJ2.setMaxSpeed(MAX_SPEED); stepJ2.setAcceleration(ACCEL);
  stepJ3.setMaxSpeed(MAX_SPEED); stepJ3.setAcceleration(ACCEL);
  steppers.addStepper(stepJ1);
  steppers.addStepper(stepJ2);
  steppers.addStepper(stepJ3);

  // Initialize current positions
  stepJ1.setCurrentPosition(lround(shoulder_deg * stepsPerDeg_J1));
  stepJ2.setCurrentPosition(lround(elbow_deg    * stepsPerDeg_J2));
  stepJ3.setCurrentPosition(lround(wrist_deg    * stepsPerDeg_J3));

  // Get current tip position
  float x, z;
  forwardKinematics(shoulder_deg, elbow_deg, x, z);
  Serial.print("Start X="); Serial.print(x); Serial.print("  Z="); Serial.println(z);

  float x_target = x + move;
  float z_target = z; // same height

  float j1_new, j2_new, j3_new;
  if (!inverseKinematics(x_target, z_target, j1_new, j2_new, j3_new)) {
    Serial.println("Target unreachable!");
    return;
  }

  Serial.println("Moving forward...");
  moveToAngles(j1_new, j2_new, j3_new);
  delay(10000);

  Serial.println("Moving back...");
  moveToAngles(shoulder_deg, elbow_deg, wrist_deg);
}

void loop() {
  // nothing
}