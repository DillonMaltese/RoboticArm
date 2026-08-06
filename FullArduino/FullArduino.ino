#include <AccelStepper.h>


// -------------------- Motor pins --------------------

#define PUL_B 47
#define DIR_B 49

#define PUL_S 51
#define DIR_S 53

#define PUL_E 35
#define DIR_E 37

#define PUL_W 39
#define DIR_W 41


// -------------------- Motor configuration --------------------

#define STEPS_PER_REV 1600.0

#define GEAR_RATIO_B 10.0
#define GEAR_RATIO_S 100.0
#define GEAR_RATIO_E 100.0
#define GEAR_RATIO_W 13.0

#define CALIBRATION_S 1.0
#define CALIBRATION_E 1.0


#define STEPS_PER_DEG_B \
    (STEPS_PER_REV * GEAR_RATIO_B / 360.0)

#define STEPS_PER_DEG_S \
    (STEPS_PER_REV * GEAR_RATIO_S / 360.0 * CALIBRATION_S)

#define STEPS_PER_DEG_E \
    (STEPS_PER_REV * GEAR_RATIO_E / 360.0 * CALIBRATION_E)

#define STEPS_PER_DEG_W \
    (STEPS_PER_REV * GEAR_RATIO_W / 360.0)


// -------------------- Python-to-motor directions --------------------

/*
   Python's URDF joint directions are opposite to the physical
   directions used in your successful motor test.
*/

#define COMMAND_SIGN_B -1.0
#define COMMAND_SIGN_S -1.0
#define COMMAND_SIGN_E -1.0
#define COMMAND_SIGN_W -1.0


// -------------------- Motor objects --------------------

AccelStepper baseMotor(
    AccelStepper::DRIVER,
    PUL_B,
    DIR_B
);

AccelStepper shoulderMotor(
    AccelStepper::DRIVER,
    PUL_S,
    DIR_S
);

AccelStepper elbowMotor(
    AccelStepper::DRIVER,
    PUL_E,
    DIR_E
);

AccelStepper wristMotor(
    AccelStepper::DRIVER,
    PUL_W,
    DIR_W
);


// -------------------- Serial input --------------------

String serialBuffer = "";


// -------------------- Step conversion --------------------

long degreesToSteps(
    float degrees,
    float stepsPerDegree
) {
    return (long)(
        degrees * stepsPerDegree
    );
}


// -------------------- Movement --------------------

/*
   Give every motor its relative movement first, and then call
   run() repeatedly for all four motors.

   This allows the motors to move during the same time period
   instead of moving base, shoulder, elbow, and wrist one at a time.
*/

void moveAllDegrees(
    float baseDegrees,
    float shoulderDegrees,
    float elbowDegrees,
    float wristDegrees
) {
    long baseSteps = degreesToSteps(
        baseDegrees,
        STEPS_PER_DEG_B
    );

    long shoulderSteps = degreesToSteps(
        shoulderDegrees,
        STEPS_PER_DEG_S
    );

    long elbowSteps = degreesToSteps(
        elbowDegrees,
        STEPS_PER_DEG_E
    );

    long wristSteps = degreesToSteps(
        wristDegrees,
        STEPS_PER_DEG_W
    );


    Serial.print("Base degrees: ");
    Serial.print(baseDegrees, 6);
    Serial.print(" | Steps: ");
    Serial.println(baseSteps);

    Serial.print("Shoulder degrees: ");
    Serial.print(shoulderDegrees, 6);
    Serial.print(" | Steps: ");
    Serial.println(shoulderSteps);

    Serial.print("Elbow degrees: ");
    Serial.print(elbowDegrees, 6);
    Serial.print(" | Steps: ");
    Serial.println(elbowSteps);

    Serial.print("Wrist degrees: ");
    Serial.print(wristDegrees, 6);
    Serial.print(" | Steps: ");
    Serial.println(wristSteps);


    // Relative movements from the current motor positions.
    baseMotor.move(baseSteps);
    shoulderMotor.move(shoulderSteps);
    elbowMotor.move(elbowSteps);
    wristMotor.move(wristSteps);


    // Keep running every motor until all four have finished.
    while (
        baseMotor.distanceToGo() != 0 ||
        shoulderMotor.distanceToGo() != 0 ||
        elbowMotor.distanceToGo() != 0 ||
        wristMotor.distanceToGo() != 0
    ) {
        baseMotor.run();
        shoulderMotor.run();
        elbowMotor.run();
        wristMotor.run();
    }
}


// -------------------- Reset --------------------

/*
   Return every motor to the step position recorded when this
   Arduino sketch started.

   This works by reversing the complete accumulated step count.
*/

void resetToStart() {
    Serial.println("RESETTING");


    long baseReturnSteps =
        -baseMotor.currentPosition();

    long shoulderReturnSteps =
        -shoulderMotor.currentPosition();

    long elbowReturnSteps =
        -elbowMotor.currentPosition();

    long wristReturnSteps =
        -wristMotor.currentPosition();


    Serial.print("Base return steps: ");
    Serial.println(baseReturnSteps);

    Serial.print("Shoulder return steps: ");
    Serial.println(shoulderReturnSteps);

    Serial.print("Elbow return steps: ");
    Serial.println(elbowReturnSteps);

    Serial.print("Wrist return steps: ");
    Serial.println(wristReturnSteps);


    // Use the same relative move technique.
    baseMotor.move(baseReturnSteps);
    shoulderMotor.move(shoulderReturnSteps);
    elbowMotor.move(elbowReturnSteps);
    wristMotor.move(wristReturnSteps);


    // Return all motors during the same time period.
    while (
        baseMotor.distanceToGo() != 0 ||
        shoulderMotor.distanceToGo() != 0 ||
        elbowMotor.distanceToGo() != 0 ||
        wristMotor.distanceToGo() != 0
    ) {
        baseMotor.run();
        shoulderMotor.run();
        elbowMotor.run();
        wristMotor.run();
    }


    // Explicitly restore the internal startup counters.
    baseMotor.setCurrentPosition(0);
    shoulderMotor.setCurrentPosition(0);
    elbowMotor.setCurrentPosition(0);
    wristMotor.setCurrentPosition(0);


    Serial.println("DONE");
}


// -------------------- MOVE command parser --------------------

/*
   Expected command:

   MOVE|base|shoulder|elbow|wrist

   Example:

   MOVE|0.000000|-6.097630|6.669985|-0.572355
*/

bool parseMoveCommand(
    String command,
    float &baseDegrees,
    float &shoulderDegrees,
    float &elbowDegrees,
    float &wristDegrees
) {
    command.trim();

    if (!command.startsWith("MOVE|")) {
        return false;
    }


    int separator1 =
        command.indexOf('|');

    int separator2 =
        command.indexOf(
            '|',
            separator1 + 1
        );

    int separator3 =
        command.indexOf(
            '|',
            separator2 + 1
        );

    int separator4 =
        command.indexOf(
            '|',
            separator3 + 1
        );


    if (
        separator1 == -1 ||
        separator2 == -1 ||
        separator3 == -1 ||
        separator4 == -1
    ) {
        return false;
    }


    String baseText = command.substring(
        separator1 + 1,
        separator2
    );

    String shoulderText = command.substring(
        separator2 + 1,
        separator3
    );

    String elbowText = command.substring(
        separator3 + 1,
        separator4
    );

    String wristText = command.substring(
        separator4 + 1
    );


    if (
        baseText.length() == 0 ||
        shoulderText.length() == 0 ||
        elbowText.length() == 0 ||
        wristText.length() == 0
    ) {
        return false;
    }


    baseDegrees =
        baseText.toFloat();

    shoulderDegrees =
        shoulderText.toFloat();

    elbowDegrees =
        elbowText.toFloat();

    wristDegrees =
        wristText.toFloat();


    return true;
}


// -------------------- MOVE command processing --------------------

void processMoveCommand(
    String command
) {
    float baseDegrees;
    float shoulderDegrees;
    float elbowDegrees;
    float wristDegrees;


    bool validCommand = parseMoveCommand(
        command,
        baseDegrees,
        shoulderDegrees,
        elbowDegrees,
        wristDegrees
    );


    if (!validCommand) {
        Serial.println(
            "ERROR|invalid command"
        );

        return;
    }


    Serial.print("Received: ");
    Serial.println(command);


    /*
       Convert Python/URDF signs into the physical directions
       established by your successful motor test.
    */

    baseDegrees *= COMMAND_SIGN_B;
    shoulderDegrees *= COMMAND_SIGN_S;
    elbowDegrees *= COMMAND_SIGN_E;
    wristDegrees *= COMMAND_SIGN_W;


    Serial.println("MOVING");


    moveAllDegrees(
        baseDegrees,
        shoulderDegrees,
        elbowDegrees,
        wristDegrees
    );


    Serial.println("DONE");
}


// -------------------- General command processing --------------------

void processCommand(
    String command
) {
    command.trim();


    if (command == "RESET") {
        resetToStart();
        return;
    }


    if (command.startsWith("MOVE|")) {
        processMoveCommand(command);
        return;
    }


    Serial.println(
        "ERROR|unknown command"
    );
}


// -------------------- Setup --------------------

void setup() {
    Serial.begin(115200);

    serialBuffer.reserve(100);


    // Base settings
    baseMotor.setPinsInverted(false);
    baseMotor.setMaxSpeed(800);
    baseMotor.setAcceleration(1000);
    baseMotor.setMinPulseWidth(3);


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


    /*
       Wherever the robot is physically positioned here becomes
       the RESET position.

       Place the arm in its proper home pose before the Arduino
       starts or resets.
    */

    baseMotor.setCurrentPosition(0);
    shoulderMotor.setCurrentPosition(0);
    elbowMotor.setCurrentPosition(0);
    wristMotor.setCurrentPosition(0);


    Serial.println("READY");
}


// -------------------- Main loop --------------------

void loop() {
    while (
        Serial.available() > 0
    ) {
        char receivedCharacter =
            Serial.read();


        // Ignore carriage-return characters.
        if (
            receivedCharacter == '\r'
        ) {
            continue;
        }


        // A newline marks the end of a command.
        if (
            receivedCharacter == '\n'
        ) {
            serialBuffer.trim();


            if (
                serialBuffer.length() > 0
            ) {
                processCommand(
                    serialBuffer
                );
            }


            serialBuffer = "";
        }
        else {
            serialBuffer +=
                receivedCharacter;


            // Prevent an excessively long command.
            if (
                serialBuffer.length() > 100
            ) {
                serialBuffer = "";

                Serial.println(
                    "ERROR|command too long"
                );
            }
        }
    }
}