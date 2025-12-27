#include <Arduino.h>
#include <math.h>
#include "KinematicSolver.h"

/* ============================================================
   CONFIGURATION
   ============================================================ */

#define NUM_AXES 6

double a[] = {0, 50, 0, 0, 0, 0};
double d[] = {50,0,0,50,0,0};
double alphas[] = {M_PI/2, 0, M_PI/2, M_PI/2, M_PI/2, 0};
double thetas[] = {0, 0, M_PI/2, -M_PI, M_PI/2, 0};

KinematicSolver robot(a, d, alphas, thetas);

/* ---------------- Stepper wiring ---------------- */
const int STEP_PIN[NUM_AXES] = {13,14,26,33,16,18};
const int DIR_PIN [NUM_AXES] = {12,27,25,32,17,19};

/* ---------------- Stepper mechanics ---------------- */
const int   STEPS_PER_REV = 25000;
const int   MICROSTEP     = 1;
const float GEAR_RATIO[NUM_AXES] = {1,1,1,1,1,1};

float STEPS_PER_JointDeg[NUM_AXES];

/* ---------------- Control timing ---------------- */
// const float Fs = 500.0f;
const float Fs =10.0f;
const float Ts = 1.0f / Fs;
const uint32_t Ts_us = (uint32_t)(1e6f * Ts);



/* ---------------- Cartesian limits ---------------- */
const float V_MAX = 1.0f;   // m/s
const float A_MAX = 3.0f;   // m/s^2

/* ============================================================
   HELPERS
   ============================================================ */

inline double deg2rad(double d) { return d * M_PI / 180.0; }

/* ============================================================
   DATA TYPES
   ============================================================ */

struct Trapezoid {
  float D, ta, tc, T, vmax, a;
};

/* ============================================================
   GLOBAL STATE
   ============================================================ */

float stepAcc[NUM_AXES] = {0};
long  prevStepTarget[NUM_AXES] = {0};

/* ============================================================
   LOW-LEVEL STEPPER
   ============================================================ */

inline void pulse(int pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);
  
}

void emitSteps(float &acc, int stepPin, int dirPin) {
  while (fabs(acc) >= 1.0f) {
    digitalWrite(dirPin, acc > 0 ? HIGH : LOW);
    pulse(stepPin);
    acc += (acc > 0) ? -1.0f : 1.0f;
  }
}

/* ============================================================
   TRAPEZOID
   ============================================================ */

Trapezoid computeTrapezoid(float D, float vmax, float a) {
  Trapezoid tr;
  tr.D = D; tr.vmax = vmax; tr.a = a;

  float ta = vmax / a;
  float da = 0.5f * a * ta * ta;

  if (2 * da <= D) {
    tr.ta = ta;
    tr.tc = (D - 2 * da) / vmax;
    tr.T  = 2 * tr.ta + tr.tc;
  } else {
    tr.vmax = sqrtf(a * D);
    tr.ta = tr.vmax / a;
    tr.tc = 0;
    tr.T = 2 * tr.ta;
  }
  return tr;
}

float currDist_of_t(const Trapezoid &tr, float t) {
  if (t <= 0) return 0;
  if (t >= tr.T) return tr.D;

  if (t < tr.ta)
    return 0.5f * tr.a * t * t;
  else if (t < tr.ta + tr.tc)
    return 0.5f * tr.a * tr.ta * tr.ta + tr.vmax * (t - tr.ta);
  else {
    float dt = tr.T - t;
    return tr.D - 0.5f * tr.a * dt * dt;
  }
}

/* ============================================================
   SETUP
   ============================================================ */

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    digitalWrite(STEP_PIN[i], LOW);
    STEPS_PER_JointDeg[i] =
      (STEPS_PER_REV/360) * MICROSTEP * GEAR_RATIO[i];
  }

  Serial.println("6-DOF Cartesian DDA planner w/ rotation");
}



/* ============================================================
   MAIN LOOP
   ============================================================ */

void loop() {

  /*Stepper test*/

  // stepAcc[0] = 25000.0f; // 2 revolutions
  // emitSteps(stepAcc[0], STEP_PIN[0], DIR_PIN[0]);
  
  
  // //  digitalWrite(DIR_PIN[0], HIGH); // Set direction
  // // digitalWrite(STEP_PIN[0], HIGH); // Pulse HIGH (e.g., 1 microsecond)
  // // delayMicroseconds(20);
  // // digitalWrite(STEP_PIN[0], LOW);  // Pulse LOW (e.g., 1 microsecond)
  // // delayMicroseconds(20);
  //  delayMicroseconds(1000); 
  
  //  Serial.println("Move complete");





  
 
  
  


  /* -------- Cartesian endpoints -------- */
  double P0[] = {50, 0, 50, 90, 0, 0};
  double P1[] = {50, 20, 50, 90, 0, 0};

  /* ---- Orientation in radians ---- */
  double R0[3] = {
    P0[3], P0[4], P0[5]
  };
  double R1[3] = {
    P1[3], P1[4], P1[5]
  };

  /* ---- Path direction ---- */
  float dx = P1[0] - P0[0];
  float dy = P1[1] - P0[1];
  float dz = P1[2] - P0[2];
  float D  = sqrt(dx*dx + dy*dy + dz*dz);

  /* ---- Unit vector along path ---- */
  float ux = dx / D;
  float uy = dy / D;
  float uz = dz / D;

  Trapezoid tr = computeTrapezoid(D, V_MAX, A_MAX);

  /* ---- Main control loop ---- */
  double q[NUM_AXES], prevQ[NUM_AXES];

  robot.solveIK(P0, true, prevQ);
  for (int i = 0; i < NUM_AXES; i++)
    prevStepTarget[i] = lround(prevQ[i] * STEPS_PER_JointDeg[i]);

  uint32_t tStart = micros();
  uint32_t nextSample = tStart;

  while (true) {
    if (micros() < nextSample) continue;
    nextSample += Ts_us;

    float t = (micros() - tStart) * 1e-6f;


    if (t > tr.T) 
      t = tr.T;

    float s = currDist_of_t(tr, t);
    double alpha = s / D;

    /* ---- Interpolated 6D pose ---- */
    double P[6];
    P[0] = P0[0] + ux * s;
    P[1] = P0[1] + uy * s;
    P[2] = P0[2] + uz * s;

    P[3] = R0[0] + alpha * (R1[0] - R0[0]);
    P[4] = R0[1] + alpha * (R1[1] - R0[1]);
    P[5] = R0[2] + alpha * (R1[2] - R0[2]);

    if (!robot.solveIK(P, true, q)) break;
    
    // Serial.println( q[0]);
    // q[0] = q[0] + 0.1;

    for (int i = 0; i < NUM_AXES; i++) {
      long targetSteps = lround(q[i] * STEPS_PER_JointDeg[i]);
      stepAcc[i] += targetSteps - prevStepTarget[i];
      prevStepTarget[i] = targetSteps;
    }
    
    uint32_t emitStart = micros();
    while (micros() - emitStart < Ts_us) {
      // for (int i = 0; i < NUM_AXES; i++)
      // {
      //   emitSteps(stepAcc[i], STEP_PIN[i], DIR_PIN[i]);
        
      // }
        for (int i = 0; i < 1; i++)
      {
        emitSteps(stepAcc[i], STEP_PIN[i], DIR_PIN[i]);
        
      }
    }

    if (t >= tr.T) 
      { Serial.println("Move complete");}
    continue;
  }

 
  while (1) delay(1000);
  
}
