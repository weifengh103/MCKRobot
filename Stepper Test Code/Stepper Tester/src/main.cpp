#include "KinematicSolver.h"

// Example DH Parameters
double a[] = {0, 50, 0, 0, 0, 0};
double d[] = {50,0,0,50,0,0};
double alphas[] = {M_PI/2, 0, M_PI/2, M_PI/2, M_PI/2, 0};
double thetas[] = {0, 0, M_PI/2, -M_PI, M_PI/2, 0};

KinematicSolver robot(a, d, alphas, thetas);

void setup() {
  Serial.begin(115200);
  
  // Target Pose: X, Y, Z, Rx, Ry, Rz
//   double targetPose[6] = {50,10,50,90,0,0};
//   double resultDegrees[6];

//   robot.solveIK(targetPose, true, resultDegrees);

//   Serial.println("IK Solution (Degrees):");
//   for(int i=0; i<6; i++) {
//     Serial.printf("Joint %d: %.2f\n", i+1, resultDegrees[i]);
//   }
    
  const int iterations = 1000;
  double targetPose[6] = {150.0, 5.0, 180.0, 10.0, 0.0, 0.0};
  double resultDegrees[6];

  Serial.println("Starting benchmark...");
  
  int64_t totalStartTime = esp_timer_get_time(); // Native ESP32 microsecond timer

  for (int i = 0; i < iterations; i++) {
    robot.solveIK(targetPose, true, resultDegrees);
  }

  int64_t totalEndTime = esp_timer_get_time();
  
  float totalDuration = (float)(totalEndTime - totalStartTime);
  float averageDuration = totalDuration / iterations;

  Serial.println("--- Benchmark Results ---");
  Serial.printf("Total time for %d runs: %.2f us\n", iterations, totalDuration);
  Serial.printf("Average time per solveIK: %.2f us\n", averageDuration);
  Serial.println("-------------------------");

}

void loop() {}