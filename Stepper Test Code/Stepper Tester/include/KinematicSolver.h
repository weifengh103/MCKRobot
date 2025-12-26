#ifndef KINEMATIC_SOLVER_H
#define KINEMATIC_SOLVER_H

#include <Arduino.h>
#include <math.h>

class KinematicSolver {
public:
    // Constructor to initialize DH parameters
    KinematicSolver(const double _a[6], const double _d[6], const double _alphas[6], const double _thetas[6]);

    // The only public operational function
    bool solveIK(const double TCPPose[6], bool elbowUp, double outDeg[6]);

private:
    double a[6], d[6], alphas[6], thetas[6];

    // Internal Math Utilities
    void eulerExtrinsicXYZ(double rx, double ry, double rz, double R[3][3]);
    void rotationMatrixToIntrinsicXYZ(const double R[3][3], double &rx, double &ry, double &rz);
    void rotationMatrixToExtrinsicXYZ(const double R[3][3], double &rx, double &ry, double &rz);
    
    void matMul4(const double A[4][4], const double B[4][4], double C[4][4]);
    void matMul3(const double A[3][3], const double B[3][3], double C[3][3]);
    void matCopy4(const double A[4][4], double B[4][4]);
    bool invert3x3(const double m[3][3], double invOut[3][3]);

    // Internal Kinematic Helpers
    void getDHTransMatrix(int i, const double angelDeg[6], double T[4][4]);
    void updateAllTJointJointTrans(const double angelDeg[6], double TJJ[6][4][4]);
    void updateAllTBaseJointTrans(double TJJ[6][4][4], double TBJ[6][4][4]);
    
    // Debugging
    void printMatrix3x3(const double M[3][3], const char* name);
    void printMatrix4x4(const double M[4][4], const char* name);
};

#endif