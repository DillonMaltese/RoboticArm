// Define direction and pulse pins for each motor
#define DIR 4
#define PUL 3

#define DIR1 12
#define PUL1 11

#define DIR2 6
#define PUL2 5

// Duration to step in each direction (in milliseconds)
unsigned long stepDuration = 3000;

// Step pulse delay (in microseconds)
unsigned int pulseDelay = 3000;

// Tracks direction state: 0 = forward, 1 = backward
bool directionState = 0;

void setup() {
  // Set all motor pins as output
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  
  pinMode(DIR1, OUTPUT);
  pinMode(PUL1, OUTPUT);

  pinMode(DIR2, OUTPUT);
  pinMode(PUL2, OUTPUT);
}

void loop() {
  // Set direction for all motors
  digitalWrite(DIR, directionState);
  digitalWrite(DIR1, directionState);
  digitalWrite(DIR2, directionState);

  // Run motors for the defined duration
  unsigned long startTime = millis();
  while (millis() - startTime < stepDuration) {
    // Pulse all motors at the same time
    //digitalWrite(PUL, HIGH);
    digitalWrite(PUL1, HIGH);
    //digitalWrite(PUL2, HIGH);
    delayMicroseconds(pulseDelay);
    
    //digitalWrite(PUL, LOW);
    digitalWrite(PUL1, LOW);
    //digitalWrite(PUL2, LOW);
    delayMicroseconds(pulseDelay);
  }

  // Toggle direction
  directionState = !directionState;

  // Optional delay before changing direction
  delay(1000);
}