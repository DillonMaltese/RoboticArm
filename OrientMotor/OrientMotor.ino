#define PUL_PIN 43
#define DIR_PIN 45

// With 1600 steps/rev and 100:1 gear ratio
#define STEPS_PER_REV     1600
#define GEAR_RATIO        100
#define STEPS_PER_DEG     (STEPS_PER_REV * GEAR_RATIO / 360.0)  // = 444.44 steps per degree

void stepMotor(long steps, bool direction) {
    digitalWrite(DIR_PIN, direction);
    delayMicroseconds(5);   // DIR must settle 5us before pulses per CL57T datasheet

    for (long i = 0; i < steps; i++) {
        digitalWrite(PUL_PIN, HIGH);
        delayMicroseconds(10);   // pulse width min 2.5us, using 10 for safety
        digitalWrite(PUL_PIN, LOW);
        delayMicroseconds(10);
    }
}

void rotateDegrees(float degrees, bool clockwise) {
    long steps = (long)(degrees * STEPS_PER_DEG);
    stepMotor(steps, clockwise);
}

void setup() {
    pinMode(PUL_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(PUL_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    
    delay(2000);   // wait for driver to initialize on power up

    // Rotate 10 degrees clockwise
    rotateDegrees(10, HIGH);
    delay(5000);
    
    // Rotate 10 degrees back
    rotateDegrees(10, LOW);
    delay(1000);
}

void loop() {
    // nothing, just runs once in setup
}