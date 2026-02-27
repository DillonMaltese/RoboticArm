// // Your wiring:
// // PUL- + DIR- + COM- -> Arduino GND
// // PUL+ -> D10
// // DIR+ -> D11
// // ENA not connected (fine)

// const int PIN_STEP = 10;   // PUL+
// const int PIN_DIR  = 11;   // DIR+
// const int PIN_ALM  = 13;   // ALM output from driver (optional)

// const unsigned int PULSE_US = 300;  // pulse width (>= 5–10us usually; 300us is very safe)
// const unsigned int GAP_US   = 300;

// void stepN(long n) {
//   for (long i = 0; i < n; i++) {
//     digitalWrite(PIN_STEP, HIGH);
//     delayMicroseconds(PULSE_US);
//     digitalWrite(PIN_STEP, LOW);
//     delayMicroseconds(GAP_US);
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   delay(300);
//   Serial.println("CL57T 1-joint test (common-cathode)");

//   pinMode(PIN_STEP, OUTPUT);
//   pinMode(PIN_DIR, OUTPUT);
//   digitalWrite(PIN_STEP, LOW);   // idle low

//   pinMode(PIN_ALM, INPUT_PULLUP); // ALM is from driver -> Arduino input
// }

// void loop() {
//   if (digitalRead(PIN_ALM) == LOW) {
//     Serial.println("ALARM ACTIVE (ALM low). Fix driver alarm first.");
//     delay(500);
//     return;
//   }

//   Serial.println("Forward...");
//   digitalWrite(PIN_DIR, HIGH);
//   stepN(2000);
//   delay(1000);

//   Serial.println("Reverse...");
//   digitalWrite(PIN_DIR, LOW);
//   stepN(2000);
//   delay(1500);
// }

// Wiring:
// PUL- + DIR- + COM- -> Arduino GND
// PUL+ -> D10
// DIR+ -> D11
// ENA+ -> 5V, ENA- -> GND

const int PIN_STEP = 10;   // PUL+
const int PIN_DIR  = 11;   // DIR+
const int PIN_ALM  = 13;   // ALM output from driver (optional)

const unsigned int PULSE_US = 1200;
const unsigned int GAP_US   = 1200;

void stepN(long n) {
  for (long i = 0; i < n; i++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(PULSE_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(GAP_US);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("CL57T base joint test");

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_STEP, LOW);

  pinMode(PIN_ALM, INPUT_PULLUP);

  Serial.println("Waiting for base to settle...");
  delay(3000);
}

void loop() {
  if (digitalRead(PIN_ALM) == LOW) {
    Serial.println("ALARM ACTIVE (ALM low)");
    delay(500);
    return;
  }

  Serial.println("Forward BIG...");
  digitalWrite(PIN_DIR, HIGH);
  stepN(20000);
  delay(1500);

  Serial.println("Reverse BIG...");
  digitalWrite(PIN_DIR, LOW);
  stepN(20000);
  delay(2500);
}