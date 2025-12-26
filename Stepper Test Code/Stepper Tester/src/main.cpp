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
const int STEP_PIN[NUM_AXES] = {2, 4, 5, 18, 19, 21};
const int DIR_PIN [NUM_AXES] = {15,16,17,22,23,25};

/* ---------------- Stepper mechanics ---------------- */
const int   STEPS_PER_REV = 200;
const int   MICROSTEP     = 16;
const float GEAR_RATIO[NUM_AXES] = {1,1,1,1,1,1};

float STEPS_PER_RAD[NUM_AXES];

/* ---------------- Control timing ---------------- */
const float Fs = 500.0f;
const float Ts = 1.0f / Fs;
const uint32_t Ts_us = (uint32_t)(1e6f * Ts);

/* ---------------- Cartesian limits ---------------- */
const float V_MAX = 0.10f;   // m/s
const float A_MAX = 0.30f;   // m/s^2

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
  delayMicroseconds(2);
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
    STEPS_PER_RAD[i] =
      (STEPS_PER_REV * MICROSTEP * GEAR_RATIO[i]) / (2.0f * PI);
  }

  Serial.println("6-DOF Cartesian DDA planner w/ rotation");
}

/* ============================================================
   MAIN LOOP
   ============================================================ */

void loop() {

  /* -------- Cartesian endpoints -------- */
  double P0[] = {0.30, 0.00, 0.30, 180, 0, 0};
  double P1[] = {0.40, 0.20, 0.25, 180, 0, 90};

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
    prevStepTarget[i] = lround(prevQ[i] * STEPS_PER_RAD[i]);

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

    for (int i = 0; i < NUM_AXES; i++) {
      long targetSteps = lround(q[i] * STEPS_PER_RAD[i]);
      stepAcc[i] += targetSteps - prevStepTarget[i];
      prevStepTarget[i] = targetSteps;
    }

    uint32_t emitStart = micros();
    while (micros() - emitStart < Ts_us) {
      for (int i = 0; i < NUM_AXES; i++)
        emitSteps(stepAcc[i], STEP_PIN[i], DIR_PIN[i]);
    }

    if (t >= tr.T) 
      { Serial.println("Move complete");}
    continue;
  }

 
  while (1) delay(1000);
}
