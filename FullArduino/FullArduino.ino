#include <AccelStepper.h>

#define PUL_B 47
#define DIR_B 49
#define PUL_S 51
#define DIR_S 53
#define PUL_E 35
#define DIR_E 37
#define PUL_W 39
#define DIR_W 41
#define PUL_R 0
#define DIR_R 0

#define STEPS_PER_REV 1600
#define GEAR_RATIO_B 10
#define GEAR_RATIO_S 100
#define GEAR_RATIO_E 100
#define GEAR_RATIO_W 13
#define GEAR_RATIO_R 13
#define CALIBRATION 5.44

#define STEPS_PER_DEG_B (STEPS_PER_REV * GEAR_RATIO_B / 360.0)
#define STEPS_PER_DEG_S (STEPS_PER_REV * GEAR_RATIO_S / 360.0 * CALIBRATION)
#define STEPS_PER_DEG_E (STEPS_PER_REV * GEAR_RATIO_E / 360.0 * CALIBRATION)
#define STEPS_PER_DEG_W (STEPS_PER_REV * GEAR_RATIO_W / 360.0)
#define STEPS_PER_DEG_R (STEPS_PER_REV * GEAR_RATIO_R / 360.0)

AccelStepper bMotor(AccelStepper::DRIVER, PUL_B, DIR_B);
AccelStepper sMotor(AccelStepper::DRIVER, PUL_S, DIR_S);
AccelStepper eMotor(AccelStepper::DRIVER, PUL_E, DIR_E);
AccelStepper wMotor(AccelStepper::DRIVER, PUL_W, DIR_W);
AccelStepper rMotor(AccelStepper::DRIVER, PUL_R, DIR_R);

String buffer = "";
bool movementActive = false;

  

void moveDegrees(float degrees, AccelStepper &motor, float stepsPerDeg) {
    long steps = (long)(degrees * stepsPerDeg);
    motor.move(steps);
    motor.runToPosition();
}


void sendCommand(String data) {
  // baseDeg,shoulderDeg,elbowDeg,wristDeg
  String sValues[4];
  int lastPlacement = 0;
  int nextPlacement = 0;
  for (int i = 0; i < 4; i++) {
    nextPlacement = data.indexOf("|", lastPlacement + 1);
    for (int p = lastPlacement + 1; p < nextPlacement; p++) sValues[i] += data[p];
    lastPlacement = nextPlacement;
  }

  float baseDeg     = sValues[0].toFloat();
  float shoulderDeg = sValues[1].toFloat();
  float elbowDeg    = sValues[2].toFloat();
  float wristDeg    = sValues[3].toFloat();

  moveDegrees(baseDeg, bMotor, STEPS_PER_DEG_B);
  moveDegrees(shoulderDeg, sMotor, STEPS_PER_DEG_S);
  moveDegrees(elbowDeg, eMotor, STEPS_PER_DEG_E);
  moveDegrees(wristDeg, wMotor, STEPS_PER_DEG_W);

}

void setup() {
  Serial.begin(115200);
    
  bMotor.setMaxSpeed(800);   bMotor.setAcceleration(1000);   bMotor.setMinPulseWidth(3); bMotor.setPinsInverted(false);
  sMotor.setMaxSpeed(1500);   sMotor.setAcceleration(2000);   sMotor.setMinPulseWidth(3); sMotor.setPinsInverted(true);
  eMotor.setMaxSpeed(1500);   eMotor.setAcceleration(2000);   eMotor.setMinPulseWidth(3); eMotor.setPinsInverted(false);
  wMotor.setMaxSpeed(1500);   wMotor.setAcceleration(2000);   wMotor.setMinPulseWidth(3); wMotor.setPinsInverted(true);
  rMotor.setMaxSpeed(1500);   rMotor.setAcceleration(2000);   rMotor.setMinPulseWidth(3); rMotor.setPinsInverted(false);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer.trim();
      if (buffer.length() > 0) {
        sendCommand(buffer);
      }
      buffer = "";
    }
    else {
      buffer += c;
    }
  }
}
