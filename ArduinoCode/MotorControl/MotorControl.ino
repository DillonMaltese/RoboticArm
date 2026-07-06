#include <AccelStepper.h>

// BASE
const int STEP_PIN = 24;   // PUL+
const int DIR_PIN  = 25;   // DIR+
const int ALM_PIN  = 32;   // ALM input (optional)
const long GEAR_RATIO = 5; // base reduction

// SHOULDER
// const int STEP_PIN = 30;     // PUL+  (shoulder from our Mega map)
// const int DIR_PIN  = 31;     // DIR+
// const int ALM_PIN  = 36;     // ALM input (optional)
// const long GEAR_RATIO = 100; // shoulder reduction

// const int STEP_PIN = 22;
// const int DIR_PIN  = 23;
// const int ALM_PIN  = 33;
// const long GEAR_RATIO = 100;

// const int STEP_PIN = 26;
// const int DIR_PIN  = 27;
// const int ALM_PIN  = 33;
// const long GEAR_RATIO = 10;


const long PULSES_PER_MOTOR_REV = 6400;   // match SW1-4
const long PULSES_PER_OUTPUT_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO;

const float MOVE_DEG = 30.0;              // output degrees

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

long degToStepsOutput(float deg) {
  return lround((deg / 360.0f) * (float)PULSES_PER_OUTPUT_REV);
}

void moveToAndWait(long targetSteps) {
  motor.moveTo(targetSteps);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ALM_PIN, INPUT_PULLUP);

  // These are motor pulse rates (not output rates).
  // Start conservative because 100:1 can make things feel "stalled" if too fast.
  // motor.setMaxSpeed(450);      // steps/sec
  // motor.setAcceleration(225);   // steps/sec^2
  motor.setMaxSpeed(10000);
  motor.setAcceleration(5000);

  motor.setCurrentPosition(0);
}

void loop() {
  if (digitalRead(ALM_PIN) == LOW) {
    Serial.println("ALARM ACTIVE");
    Serial.println(digitalRead(ALM_PIN));
    delay(500);
    return;
  }

  long d = degToStepsOutput(MOVE_DEG);

  moveToAndWait(+d);
  delay(500);

  //moveToAndWait(0);
  delay(500);

  moveToAndWait(+d);
  delay(500);

  //moveToAndWait(0);
  delay(1000);
}