// ── Pin Definitions ──────────────────────────────────────────────────────────
#define BASE_PUL      0   // TODO: fill in
#define BASE_DIR      0   // TODO: fill in

#define SHOULDER_PUL  0   // TODO: fill in
#define SHOULDER_DIR  0   // TODO: fill in

#define ELBOW_PUL     0   // TODO: fill in
#define ELBOW_DIR     0   // TODO: fill in

#define WRIST_PUL     0   // TODO: fill in
#define WRIST_DIR     0   // TODO: fill in

#define WRISTROT_PUL  0   // TODO: fill in
#define WRISTROT_DIR  0   // TODO: fill in

// ── Motor Config ──────────────────────────────────────────────────────────────
#define STEPS_PER_REV     1600
#define GEAR_RATIO        100
#define STEPS_PER_DEG     (STEPS_PER_REV * GEAR_RATIO / 360.0)  // 444.44

#define TEST_DEGREES      30
#define DELAY_BETWEEN_MS  1000   // pause between each joint test

// ── Motor Control ─────────────────────────────────────────────────────────────
void stepMotor(int pulPin, int dirPin, long steps, bool clockwise) {
    digitalWrite(dirPin, clockwise);
    delayMicroseconds(5);   // DIR must settle 5us per CL57T datasheet

    for (long i = 0; i < steps; i++) {
        digitalWrite(pulPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(pulPin, LOW);
        delayMicroseconds(10);
    }
}

void rotateDegrees(int pulPin, int dirPin, float degrees, bool clockwise) {
    long steps = (long)(degrees * STEPS_PER_DEG);
    stepMotor(pulPin, dirPin, steps, clockwise);
}

void testJoint(const char* name, int pulPin, int dirPin) {
    Serial.print("Testing: ");
    Serial.println(name);

    Serial.println("  -> 30 deg CW");
    rotateDegrees(pulPin, dirPin, TEST_DEGREES, HIGH);
    delay(DELAY_BETWEEN_MS);

    Serial.println("  -> 30 deg CCW");
    rotateDegrees(pulPin, dirPin, TEST_DEGREES, LOW);
    delay(DELAY_BETWEEN_MS);

    Serial.println("  -> Done");
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Set all pins as outputs
    int pins[] = {
        BASE_PUL, BASE_DIR,
        SHOULDER_PUL, SHOULDER_DIR,
        ELBOW_PUL, ELBOW_DIR,
        WRIST_PUL, WRIST_DIR,
        WRISTROT_PUL, WRISTROT_DIR
    };
    for (int i = 0; i < 10; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }

    delay(2000);   // wait for all drivers to initialize
    Serial.println("Starting joint movement test...");
    Serial.println("Each joint will rotate 30 degrees CW then 30 degrees CCW");
    Serial.println("----------------------------------------");

    testJoint("Base",         BASE_PUL,     BASE_DIR);
    testJoint("Shoulder",     SHOULDER_PUL, SHOULDER_DIR);
    testJoint("Elbow",        ELBOW_PUL,    ELBOW_DIR);
    testJoint("Wrist",        WRIST_PUL,    WRIST_DIR);
    testJoint("Wrist Rotate", WRISTROT_PUL, WRISTROT_DIR);

    Serial.println("----------------------------------------");
    Serial.println("All joints tested.");
}

void loop() {
    // nothing, runs once
}