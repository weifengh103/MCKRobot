

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

// 

// int pulseDelay = 12000;   // Microseconds between pulses
int pulseDelay = 200; 
int steps = 30000;        // Number of steps per move
String inputString = ""; // Buffer for serial input

void setup() {
  pinMode(PUL1, OUTPUT);
  pinMode(DIR1, OUTPUT);

  pinMode(PUL2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  pinMode(PUL3, OUTPUT);
  pinMode(DIR3, OUTPUT);

  pinMode(PUL4, OUTPUT);
  pinMode(DIR4, OUTPUT);

  pinMode(PUL5, OUTPUT);
  pinMode(DIR5, OUTPUT);

  pinMode(PUL6, OUTPUT);
  pinMode(DIR6, OUTPUT);

  // // Enable the driver (active LOW for most drivers)
  // digitalWrite(ENA, LOW);

  Serial.begin(115200);
  Serial.println("Type 'start' to move the motor...");
}

void loop()
{
  // Check if data is available on Serial
  if (Serial.available() > 0) 
  {
    inputString = Serial.readStringUntil('\n'); // Read input until newline
    inputString.trim(); // Remove spaces/newlines

    if (inputString.equalsIgnoreCase("1")) 
    {
      Serial.println("Motor 1 running...");

      // Forward
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, HIGH);
      digitalWrite(DIR3, HIGH);
      digitalWrite(DIR4, HIGH);
      digitalWrite(DIR5, HIGH);
      digitalWrite(DIR6, HIGH);
      for (int i = 0; i < steps; i++)
      {
        digitalWrite(PUL1, HIGH);
        digitalWrite(PUL2, HIGH);
        digitalWrite(PUL3, HIGH);
        digitalWrite(PUL4, HIGH);
        digitalWrite(PUL5, HIGH);
        digitalWrite(PUL6, HIGH);

        delayMicroseconds(pulseDelay);
        digitalWrite(PUL1, LOW);
        digitalWrite(PUL2, LOW);
        digitalWrite(PUL3, LOW);
        digitalWrite(PUL4, LOW);
        digitalWrite(PUL5, LOW);
        digitalWrite(PUL6, LOW);
        delayMicroseconds(pulseDelay);
      }

      delay(500);  // Pause between directions

      // Backward
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, LOW);
      digitalWrite(DIR3, LOW);
      digitalWrite(DIR4, LOW);
      digitalWrite(DIR5, LOW);
      digitalWrite(DIR6, LOW);
      for (int i = 0; i < steps; i++) 
      {
          digitalWrite(PUL1, HIGH);
          digitalWrite(PUL2, HIGH);
          digitalWrite(PUL3, HIGH);
          digitalWrite(PUL4, HIGH);
          digitalWrite(PUL5, HIGH);
          digitalWrite(PUL6, HIGH);

          delayMicroseconds(pulseDelay);
          digitalWrite(PUL1, LOW);
          digitalWrite(PUL2, LOW);
          digitalWrite(PUL3, LOW);
          digitalWrite(PUL4, LOW);
          digitalWrite(PUL5, LOW);
          digitalWrite(PUL6, LOW);
          delayMicroseconds(pulseDelay);
      }
    }
    delay(500);  // Pause before accepting new command
    Serial.println("Done. Type 'start' again to move.");
    
  }
}

// #include <Arduino.h>
// #include <AccelStepper.h>

// // --- Motor Pin Definitions ---

// // Join1 
// #define PUL1 25   // Pulse pin
// #define DIR1 26   // Direction pin

// // Join2 
// #define PUL2 27   // Pulse pin
// #define DIR2 14   // Direction pin

// // Join3 
// #define PUL3 12   // Pulse pin
// #define DIR3 13   // Direction pin

// // Join4 
// #define PUL4 16   // Pulse pin
// #define DIR4 17   // Direction pin

// // Join5 
// #define PUL5 18   // Pulse pin
// #define DIR5 19   // Direction pin

// // Join6 
// #define PUL6 32   // Pulse pin
// #define DIR6 33   // Direction pin

// // --- Create AccelStepper objects ---
// AccelStepper stepper1(AccelStepper::DRIVER, PUL1, DIR1);
// AccelStepper stepper2(AccelStepper::DRIVER, PUL2, DIR2);
// AccelStepper stepper3(AccelStepper::DRIVER, PUL3, DIR3);
// AccelStepper stepper4(AccelStepper::DRIVER, PUL4, DIR4);
// AccelStepper stepper5(AccelStepper::DRIVER, PUL5, DIR5);
// AccelStepper stepper6(AccelStepper::DRIVER, PUL6, DIR6);

// // --- Serial Input ---
// String inputString = "";

// // --- Motion Settings ---
// const int maxSpeed = 50;     // steps per second
// const int accel    = 10;     // steps per second^2
// const int steps    = 1500;   // steps per move

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Type '1'..'6' for individual motor, or 'all' to move all together.");

//   // Configure all steppers
//   stepper1.setMaxSpeed(maxSpeed);
//   stepper1.setAcceleration(accel);

//   stepper2.setMaxSpeed(maxSpeed);
//   stepper2.setAcceleration(accel);

//   stepper3.setMaxSpeed(maxSpeed);
//   stepper3.setAcceleration(accel);

//   stepper4.setMaxSpeed(maxSpeed);
//   stepper4.setAcceleration(accel);

//   stepper5.setMaxSpeed(maxSpeed);
//   stepper5.setAcceleration(accel);

//   stepper6.setMaxSpeed(maxSpeed);
//   stepper6.setAcceleration(accel);
// }

// void runStepper(AccelStepper &stepper, int distance) {
//   stepper.move(distance);
//   while (stepper.isRunning()) {
//     stepper.run();
//   }
// }

// void loop() {
//   // --- Read Serial Command ---
//   if (Serial.available() > 0) {
//     inputString = Serial.readStringUntil('\n');
//     inputString.trim();

//     // === Individual Motor Control ===
//     if (inputString == "1") {
//       Serial.println("Motor 1 moving forward/backward...");
//       runStepper(stepper1, steps);
//       delay(500);
//       runStepper(stepper1, -steps);
//       Serial.println("Motor 1 done.");
//     }
//     else if (inputString == "2") {
//       Serial.println("Motor 2 moving forward/backward...");
//       runStepper(stepper2, steps);
//       delay(500);
//       runStepper(stepper2, -steps);
//       Serial.println("Motor 2 done.");
//     }
//     else if (inputString == "3") {
//       Serial.println("Motor 3 moving forward/backward...");
//       runStepper(stepper3, steps);
//       delay(500);
//       runStepper(stepper3, -steps);
//       Serial.println("Motor 3 done.");
//     }
//     else if (inputString == "4") {
//       Serial.println("Motor 4 moving forward/backward...");
//       runStepper(stepper4, steps);
//       delay(500);
//       runStepper(stepper4, -steps);
//       Serial.println("Motor 4 done.");
//     }
//     else if (inputString == "5") {
//       Serial.println("Motor 5 moving forward/backward...");
//       runStepper(stepper5, steps);
//       delay(500);
//       runStepper(stepper5, -steps);
//       Serial.println("Motor 5 done.");
//     }
//     else if (inputString == "6") {
//       Serial.println("Motor 6 moving forward/backward...");
//       runStepper(stepper6, steps);
//       delay(500);
//       runStepper(stepper6, -steps);
//       Serial.println("Motor 6 done.");
//     }

//     // === All Motors Together ===
//     else if (inputString.equalsIgnoreCase("all")) {
//       Serial.println("Moving ALL motors together...");

//       // Forward move
//       stepper1.move(steps);
//       stepper2.move(steps);
//       stepper3.move(steps);
//       stepper4.move(steps);
//       stepper5.move(steps);
//       stepper6.move(steps);

//       while (stepper1.isRunning() || stepper2.isRunning() ||
//              stepper3.isRunning() || stepper4.isRunning() ||
//              stepper5.isRunning() || stepper6.isRunning()) {
//         stepper1.run();
//         stepper2.run();
//         stepper3.run();
//         stepper4.run();
//         stepper5.run();
//         stepper6.run();
//       }

//       delay(500);

//       // Backward move
//       stepper1.move(-steps);
//       stepper2.move(-steps);
//       stepper3.move(-steps);
//       stepper4.move(-steps);
//       stepper5.move(-steps);
//       stepper6.move(-steps);

//       while (stepper1.isRunning() || stepper2.isRunning() ||
//              stepper3.isRunning() || stepper4.isRunning() ||
//              stepper5.isRunning() || stepper6.isRunning()) {
//         stepper1.run();
//         stepper2.run();
//         stepper3.run();
//         stepper4.run();
//         stepper5.run();
//         stepper6.run();
//       }

//       Serial.println("All motors done.");
//     }
//   }
// }
