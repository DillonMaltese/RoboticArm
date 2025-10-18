const int PIN_STEP1 = 2;
const int PIN_DIR1  = 3;
const int PIN_BRK1  = 4; // brake relay
const int PIN_ALM1  = 5; // alarm

const int PIN_STEP2 = 6;
const int PIN_DIR2  = 7;
const int PIN_BRK2  = 8; // brake relay
const int PIN_ALM2  = 9; // alarm

const int PIN_STEP3 = 10;
const int PIN_DIR3  = 11;
const int PIN_BRK3  = 12; // brake relay
const int PIN_ALM3  = 13; // alarm

const int STEPS_PER_REV = 3200;  // matches DIP

void setup() {
  pinMode(PIN_STEP1, OUTPUT);
  pinMode(PIN_DIR1,  OUTPUT);
  pinMode(PIN_BRK1,  OUTPUT);
  pinMode(PIN_ALM1,  INPUT_PULLUP);

  pinMode(PIN_STEP2, OUTPUT);
  pinMode(PIN_DIR2,  OUTPUT);
  pinMode(PIN_BRK2,  OUTPUT);
  pinMode(PIN_ALM2,  INPUT_PULLUP);

  pinMode(PIN_STEP3, OUTPUT);
  pinMode(PIN_DIR3,  OUTPUT);
  pinMode(PIN_BRK3,  OUTPUT);
  pinMode(PIN_ALM3,  INPUT_PULLUP);

  // release brakes (invert HIGH/LOW if your relay is low-level trigger)
  digitalWrite(PIN_BRK1, HIGH);
  digitalWrite(PIN_BRK2, HIGH);
  digitalWrite(PIN_BRK3, HIGH);
  delay(300);
}

static void stepN(int stepPin, int dirPin, bool dir, int n) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  for (int i = 0; i < n; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(750);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(750); // ~333 Hz
  }
}

void loop() {
  // Motor 1: forward & reverse
  // stepN(PIN_STEP1, PIN_DIR1, true,  STEPS_PER_REV);
  // delay(1000);
  // stepN(PIN_STEP1, PIN_DIR1, false, STEPS_PER_REV);
  // stepN(PIN_STEP1, PIN_DIR1, false, STEPS_PER_REV);
  // delay(1000);
  // stepN(PIN_STEP1, PIN_DIR1, true,  STEPS_PER_REV);


  // // Motor 3: forward & reverse

  // stepN(PIN_STEP3, PIN_DIR3, true, STEPS_PER_REV);
  // delay(1000);

  // // Motor 2: forward & reverse
  // stepN(PIN_STEP2, PIN_DIR2, true,  STEPS_PER_REV);
  // delay(1000);
  stepN(PIN_STEP3, PIN_DIR3, false,  STEPS_PER_REV / 3);
  delay(1000);
  stepN(PIN_STEP2, PIN_DIR2, true, STEPS_PER_REV);
  
  // delay(2000);
  // stepN(PIN_STEP2, PIN_DIR3, false, STEPS_PER_REV / 3);
  // delay(1000);
  // stepN(PIN_STEP3, PIN_DIR2, true,  STEPS_PER_REV);
  delay(6000);
}