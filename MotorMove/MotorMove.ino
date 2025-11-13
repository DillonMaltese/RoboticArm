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

int STEPS_PER_5DEG_J1 = 200;  // shoulder placeholder
int STEPS_PER_5DEG_J2 = 200;  // elbow placeholder
int STEPS_PER_5DEG_J3 = 200;  // wrist placeholder

float j1_deg_curr = 0.0;
float j2_deg_curr = 0.0;
float j3_deg_curr = 0.0;


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
    delayMicroseconds(1000);
  }
}

static int degToSteps(float deg, int steps_per_5deg) {
  // Placeholder: linear scale from "steps per 5 deg"
  float steps = (deg / 5.0f) * steps_per_5deg;
  return (int)roundf(steps);
}

static void moveJointToDeg(int joint, float target_deg) {
  // choose pins & scale
  int stepPin, dirPin, steps_per_5deg;
  float *p_curr;

  if (joint == 1) { stepPin = PIN_STEP3; dirPin = PIN_DIR3; steps_per_5deg = STEPS_PER_5DEG_J1; p_curr = &j1_deg_curr; }
  else if (joint == 2) { stepPin = PIN_STEP2; dirPin = PIN_DIR2; steps_per_5deg = STEPS_PER_5DEG_J2; p_curr = &j2_deg_curr; }
  else { stepPin = PIN_STEP1; dirPin = PIN_DIR1; steps_per_5deg = STEPS_PER_5DEG_J3; p_curr = &j3_deg_curr; }

  float delta_deg = target_deg - (*p_curr);
  bool dir = (delta_deg >= 0);
  int steps = degToSteps(fabs(delta_deg), steps_per_5deg);

  stepN(stepPin, dirPin, dir, steps);
  *p_curr = target_deg;
}

// ======= MAIN LOOP =======
String cmd;
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd.trim();
      if (cmd.startsWith("JSET")) {
        // Format: JSET <J1_deg> <J2_deg> <J3_deg>
        float a, b, cdeg;
        int n = sscanf(cmd.c_str(), "JSET %f %f %f", &a, &b, &cdeg);
        if (n == 3) {
          moveJointToDeg(1, a);  // shoulder
          moveJointToDeg(2, b);  // elbow
          moveJointToDeg(3, cdeg); // wrist
          Serial.println("OK");
        } else {
          Serial.println("ERR BADARGS");
        }
      } else if (cmd.equalsIgnoreCase("HELLO")) {
        // simple wave/demo
        moveJointToDeg(3, j3_deg_curr + 10);
        moveJointToDeg(3, j3_deg_curr - 10);
        moveJointToDeg(3, j3_deg_curr);
        Serial.println("OK");
      } else {
        Serial.println("ERR UNKNOWN");
      }
      cmd = "";
    } else {
      cmd += c;
    }
  }
}