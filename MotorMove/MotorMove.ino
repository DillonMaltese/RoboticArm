// ======= PINS =======
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

const int STEPS_PER_REV = 3200;  // matches DIP


// ======= SETUP =======
void setup() {
  Serial.begin(115200);

  pinMode(PIN_STEP1, OUTPUT); pinMode(PIN_DIR1, OUTPUT);
  pinMode(PIN_STEP2, OUTPUT); pinMode(PIN_DIR2, OUTPUT);
  pinMode(PIN_STEP3, OUTPUT); pinMode(PIN_DIR3, OUTPUT);
  pinMode(PIN_BRK1, OUTPUT);  pinMode(PIN_BRK2, OUTPUT);  pinMode(PIN_BRK3, OUTPUT);
  pinMode(PIN_ALM1, INPUT_PULLUP); pinMode(PIN_ALM2, INPUT_PULLUP); pinMode(PIN_ALM3, INPUT_PULLUP);

  // release brakes
  digitalWrite(PIN_BRK1, HIGH);
  digitalWrite(PIN_BRK2, HIGH);
  digitalWrite(PIN_BRK3, HIGH);
  delay(300);

  Serial.println("OK READY");
}

static void stepN(int stepPin, int dirPin, bool dir, int n) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  for (int i = 0; i < n; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000); // ~333 Hz
  }
}

// ======= MAIN LOOP =======
String cmd;
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd.trim();
      if (cmd.equalsIgnoreCase("FORWARD 3")) {
        stepN(PIN_STEP2, PIN_DIR2, false, STEPS_PER_REV);
        delay(1000);
        stepN(PIN_STEP3, PIN_DIR3, true,  STEPS_PER_REV / 3);
      } else {
        Serial.print("idk");
      }
      cmd = "";
    } else {
      cmd += c;
    }
  }   // <-- closes while
}     // <-- closes loop

