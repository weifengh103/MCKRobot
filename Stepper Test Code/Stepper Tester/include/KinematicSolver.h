// ------------------------------
// File: KinematicSolver.h
// ------------------------------

#ifndef ESP32_KINEMATIC_SOLVER_H
#define ESP32_KINEMATIC_SOLVER_H

// Required standard headers
#include <Arduino.h>
#include <math.h>

// Constants
#define DOF 6 // Degrees of Freedom
#define MAT 4 // Matrix size (4x4)

// Macro to ensure a value is within a range (borrowed from Arduino)
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// ===================== 4x4 Matrix Class (Mostly inline in header) =====================
class Matrix4x4 {
public:
    double m[MAT][MAT];

    Matrix4x4(); // Constructor Declaration

    // Operator overloading and helpers kept inline for efficiency/simplicity
    Matrix4x4 operator*(const Matrix4x4 &o) const;
    Matrix4x4 inverseRT() const;
    
    // Add a simple print function for debugging (Declaration)
    void print();
};

// ===================== Rotation Helpers (Inline) =====================
// Kept inline in the header for compiler optimization and accessibility
// These are not part of the class, they are global helpers.
void eulerXYZ(double rx, double ry, double rz, double R[3][3]);
void rotToEulerXYZ(double R[3][3], double e[3]);

// ===================== Kinematic Solver Class (Declarations) =====================
// Only function signatures are placed here.
class KinematicSolver {
public:
    double a[DOF], d[DOF], alpha[DOF], theta[DOF];

    // Constructor declaration
    KinematicSolver(double *_a, double *_d, double *_alpha, double *_theta);

    // Forward Kinematics (FK) and Inverse Kinematics (IK) declarations
    Matrix4x4 dh(int i, double *qDeg);
    Matrix4x4 FK(double *qDeg, double *tcpPose);
    void IK(double *pose, bool elbowUp, double *qDeg);
};

#endif // ESP32_KINEMATIC_SOLVER_H