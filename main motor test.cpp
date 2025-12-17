#include <Arduino.h>

// Join1 
int PUL1 = 25;   // Pulse pin
int DIR1 = 26;   // Direction pin

// Join2 
int PUL2 = 27;   // Pulse pin
int DIR2 = 14;   // Direction pin

// Join3 
int PUL3 = 12;   // Pulse pin
int DIR3 = 13;   // Direction pin

// Join4 
int PUL4 = 16;   // Pulse pin
int DIR4 = 17;   // Direction pin

// Join5 
int PUL5 = 18;   // Pulse pin
int DIR5 = 19;   // Direction pin

// Join6 
int PUL6 = 32;   // Pulse pin
int DIR6 = 33;   // Direction pin

int pulseDelayMin = 200;    // Fastest speed (us)
int pulseDelayMax = 1000;   // Slowest speed (us)
int steps = 30000;          // Number of steps per move
int accelSteps = 1000;      // Steps for acceleration/deceleration
String inputString = "";    // Buffer for serial input

void setup() {
  pinMode(PUL1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(PUL2, OUTPUT); pinMode(DIR2, OUTPUT);
  pinMode(PUL3, OUTPUT); pinMode(DIR3, OUTPUT);
  pinMode(PUL4, OUTPUT); pinMode(DIR4, OUTPUT);
  pinMode(PUL5, OUTPUT); pinMode(DIR5, OUTPUT);
  pinMode(PUL6, OUTPUT); pinMode(DIR6, OUTPUT);

  Serial.begin(115200);
  Serial.println("Type '1' to move the motors...");
}

// Simple linear ramp for acceleration/deceleration
int getPulseDelay(int step, int totalSteps, int accelSteps, int minDelay, int maxDelay) {
  if (step < accelSteps) {
    // Accelerating
    return maxDelay - ((maxDelay - minDelay) * step) / accelSteps;
  } else if (step > totalSteps - accelSteps) {
    // Decelerating
    return maxDelay - ((maxDelay - minDelay) * (totalSteps - step)) / accelSteps;
  } else {
    // Constant speed
    return minDelay;
  }
}

void moveMotors(bool forward) {
  // Set direction
  digitalWrite(DIR1, forward ? HIGH : LOW);
  digitalWrite(DIR2, forward ? HIGH : LOW);
  digitalWrite(DIR3, forward ? HIGH : LOW);
  digitalWrite(DIR4, forward ? HIGH : LOW);
  digitalWrite(DIR5, forward ? HIGH : LOW);
  digitalWrite(DIR6, forward ? HIGH : LOW);

  for (int i = 0; i < steps; i++) {
    int delayUs = getPulseDelay(i, steps, accelSteps, pulseDelayMin, pulseDelayMax);

    digitalWrite(PUL1, HIGH); digitalWrite(PUL2, HIGH); digitalWrite(PUL3, HIGH);
    digitalWrite(PUL4, HIGH); digitalWrite(PUL5, HIGH); digitalWrite(PUL6, HIGH);

    delayMicroseconds(delayUs);

    digitalWrite(PUL1, LOW); digitalWrite(PUL2, LOW); digitalWrite(PUL3, LOW);
    digitalWrite(PUL4, LOW); digitalWrite(PUL5, LOW); digitalWrite(PUL6, LOW);

    delayMicroseconds(delayUs);
  }
}

void loop() {
  if (Serial.available() > 0) {
    inputString = Serial.readStringUntil('\n');
    inputString.trim();

    if (inputString.equalsIgnoreCase("1")) {
      Serial.println("Motors running with acceleration/deceleration...");

      // moveMotors(true);   // Forward
      // delay(1500);
      // moveMotors(false);  // Backward
      // delay(1500);
      // Serial.println("Done. Type '1' again to move.");

      for (int j = 0; j < 5; j++) { // Repeat 3 times
        moveMotors(true);   // Forward
        delay(1500);
        moveMotors(false);  // Backward
        delay(1500);
      }

    }
   
  }
}
