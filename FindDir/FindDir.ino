#include <AccelStepper.h>

#define PUL_S 51
#define DIR_S 53

#define PUL_E 35
#define DIR_E 37

#define PUL_W 39
#define DIR_W 41

#define STEPS_PER_REV   1600
#define GEAR_RATIO_S 100.0
#define GEAR_RATIO_E 100.0
#define GEAR_RATIO_W 10.0

#define CALIBRATION_S 1
#define CALIBRATION_E 1

#define STEPS_PER_DEG_S (STEPS_PER_REV * GEAR_RATIO_S / 360.0 * CALIBRATION_S)
#define STEPS_PER_DEG_E (STEPS_PER_REV * GEAR_RATIO_E / 360.0 * CALIBRATION_E)
#define STEPS_PER_DEG_W (STEPS_PER_REV * GEAR_RATIO_W / 360.0)

#define MOVE_SHOULDER_DEG  -2
#define MOVE_ELBOW_DEG      0
#define MOVE_WRIST_DEG     10

AccelStepper shoulderMotor(AccelStepper::DRIVER, PUL_S, DIR_S);
AccelStepper elbowMotor(AccelStepper::DRIVER, PUL_E, DIR_E);
AccelStepper wristMotor(AccelStepper::DRIVER, PUL_W, DIR_W);

void moveDegrees(float degrees, AccelStepper &motor, float STEPS_PER_DEG) {
    long steps = (long)(degrees * STEPS_PER_DEG);
    Serial.print("Steps: ");
    Serial.println(steps);
    motor.move(steps);
    motor.runToPosition();
}

void setup() {
    Serial.begin(115200);
    // Shoulder settings
    shoulderMotor.setPinsInverted(true);
    shoulderMotor.setMaxSpeed(1500);
    shoulderMotor.setAcceleration(2000);
    shoulderMotor.setMinPulseWidth(3);


    // Elbow settings
    elbowMotor.setPinsInverted(false);
    elbowMotor.setMaxSpeed(1500);
    elbowMotor.setAcceleration(2000);
    elbowMotor.setMinPulseWidth(3);


    // Wrist settings
    wristMotor.setPinsInverted(true);
    wristMotor.setMaxSpeed(1500);
    wristMotor.setAcceleration(2000);
    wristMotor.setMinPulseWidth(3);

    delay(2000);

    Serial.println("Shoulder:");
    // moveDegrees(
    //     MOVE_SHOULDER_DEG,
    //     shoulderMotor,
    //     STEPS_PER_DEG_S
    // );

    // Serial.println("Elbow:");
    // moveDegrees(
    //     MOVE_ELBOW_DEG,
    //     elbowMotor,
    //     STEPS_PER_DEG_E
    // );

    Serial.println("Wrist:");
    moveDegrees(
        MOVE_WRIST_DEG,
        wristMotor,
        STEPS_PER_DEG_W
    );


    // -------------------- Hold --------------------

    Serial.println("Holding for 5 seconds");

    delay(5000);


    // -------------------- Return --------------------

    Serial.println("Returning to the starting position");

    Serial.println("Wrist:");
    moveDegrees(
        -MOVE_WRIST_DEG,
        wristMotor,
        STEPS_PER_DEG_W
    );

    // Serial.println("Elbow:");
    // moveDegrees(
    //     -MOVE_ELBOW_DEG,
    //     elbowMotor,
    //     STEPS_PER_DEG_E
    // );

    // Serial.println("Shoulder:");
    // moveDegrees(
    //     -MOVE_SHOULDER_DEG,
    //     shoulderMotor,
    //     STEPS_PER_DEG_S
    // );
}

void loop() {
}