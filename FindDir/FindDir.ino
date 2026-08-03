#include <AccelStepper.h>

#define PUL_PIN 51
#define DIR_PIN 53

#define STEPS_PER_REV   1600
#define GEAR_RATIO      100
#define CALIBRATION     5.44    // measured correction factor
// #define CALIBRATION     1    // measured correction factor
#define STEPS_PER_DEG   (STEPS_PER_REV * GEAR_RATIO / 360.0 * CALIBRATION)

AccelStepper motor(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

void moveDegrees(float degrees) {
    long steps = (long)(degrees * STEPS_PER_DEG);
    Serial.print("Steps: ");
    Serial.println(steps);
    motor.move(steps);
    motor.runToPosition();
}

void setup() {
    Serial.begin(115200);
    motor.setPinsInverted(true);
    motor.setMaxSpeed(1500);
    motor.setAcceleration(2000);
    motor.setMinPulseWidth(3);
    delay(2000);
}

void loop() {
    Serial.println("30 CW");
    moveDegrees(-30);
    delay(2000);

    Serial.println("30 CCW");
    moveDegrees(-20);
    delay(2000);
}