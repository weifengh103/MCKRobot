#include <Arduino.h>
#include "KinematicSolver.h"

float a[6]      = {0,50,0,0,0,0};
float d[6]      = {50,0,0,50,0,0};
float alpha[6]  = {M_PI/2, 0, M_PI/2, M_PI/2, M_PI/2, 0};
float theta0[6] = {0, 0, M_PI/2, -M_PI, M_PI/2, 0};

KinematicSolver solver(a, d, alpha, theta0);

void setup() {
  Serial.begin(115200);
}

void loop() {
  float tcpPose[6] = {50,0,50,180,0,0};
  float joints[6];

  solver.SolveIK(tcpPose, true, joints);

  // for (int i = 0; i < 6; i++) {
  //   Serial.print("J");
  //   Serial.print(i + 1);
  //   Serial.print(": ");
  //   Serial.println(joints[i]);
  // }

  delay(1000);
}
