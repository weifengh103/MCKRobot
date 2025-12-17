// ------------------------------
// File: KinematicSolver.cpp
// ------------------------------

// Must include its own header file
#include "KinematicSolver.h"

// ===================== Matrix4x4 Definitions =====================

// Constructor Definition
Matrix4x4::Matrix4x4() {
    for (int i = 0; i < MAT; i++)
        for (int j = 0; j < MAT; j++)
            m[i][j] = (i == j) ? 1.0 : 0.0;
}

// Operator* Definition
Matrix4x4 Matrix4x4::operator*(const Matrix4x4 &o) const {
    Matrix4x4 r;
    for (int i = 0; i < MAT; i++)
        for (int j = 0; j < MAT; j++) {
            r.m[i][j] = 0;
            for (int k = 0; k < MAT; k++)
                r.m[i][j] += m[i][k] * o.m[k][j];
        }
    return r;
}

// inverseRT Definition
Matrix4x4 Matrix4x4::inverseRT() const {
    Matrix4x4 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = m[j][i];

    for (int i = 0; i < 3; i++)
        r.m[i][3] = -(r.m[i][0]*m[0][3] + r.m[i][1]*m[1][3] + r.m[i][2]*m[2][3]);

    return r;
}

// Simple print function definition for debugging
void Matrix4x4::print() {
    Serial.println("Matrix 4x4:");
    for (int i = 0; i < MAT; i++) {
        Serial.printf("| %10.4f %10.4f %10.4f %10.4f |\n", m[i][0], m[i][1], m[i][2], m[i][3]);
    }
}


// ===================== Rotation Helpers Definitions =====================
// Note: These are NOT part of the class, so they don't need the KinematicSolver:: prefix.

void eulerXYZ(double rx, double ry, double rz, double R[3][3]) {
    double cx = cos(rx), sx = sin(rx);
    double cy = cos(ry), sy = sin(ry);
    double cz = cos(rz), sz = sin(rz);

    R[0][0] = cy*cz;    R[0][1] = -cy*sz;  R[0][2] = sy;
    R[1][0] = cx*sz + sx*sy*cz;
    R[1][1] = cx*cz - sx*sy*sz;
    R[1][2] = -sx*cy;
    R[2][0] = sx*sz - cx*sy*cz;
    R[2][1] = sx*cz + cx*sy*sz;
    R[2][2] = cx*cy;
}

void rotToEulerXYZ(double R[3][3], double e[3]) {
    e[1] = asin(R[0][2]);
    e[0] = atan2(-R[1][2], R[2][2]);
    e[2] = atan2(-R[0][1], R[0][0]);
}


// ===================== Kinematic Solver Definitions =====================

// Constructor Definition
KinematicSolver::KinematicSolver(double *_a, double *_d, double *_alpha, double *_theta) {
    for (int i = 0; i < DOF; i++) {
        a[i] = _a[i];
        d[i] = _d[i];
        alpha[i] = _alpha[i];
        theta[i] = _theta[i];
    }
}

// dh (Denavit-Hartenberg) Definition
Matrix4x4 KinematicSolver::dh(int i, double *qDeg) {
    Matrix4x4 T;
    double t = theta[i] + qDeg[i] * DEG_TO_RAD;
    double ct = cos(t), st = sin(t);
    double ca = cos(alpha[i]), sa = sin(alpha[i]);

    T.m[0][0] = ct;  T.m[0][1] = -st*ca;  T.m[0][2] = st*sa;  T.m[0][3] = a[i]*ct;
    T.m[1][0] = st;  T.m[1][1] = ct*ca;   T.m[1][2] = -ct*sa; T.m[1][3] = a[i]*st;
    T.m[2][0] = 0;   T.m[2][1] = sa;      T.m[2][2] = ca;     T.m[2][3] = d[i];
    return T;
}

// FK (Forward Kinematics) Definition
Matrix4x4 KinematicSolver::FK(double *qDeg, double *tcpPose) {
    Matrix4x4 T;
    for (int i = 0; i < DOF; i++) T = T * dh(i, qDeg);

    Matrix4x4 TTCP;
    double R[3][3];
    eulerXYZ(tcpPose[3]*DEG_TO_RAD, tcpPose[4]*DEG_TO_RAD, tcpPose[5]*DEG_TO_RAD, R);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            TTCP.m[i][j] = R[i][j];
    TTCP.m[0][3] = tcpPose[0];
    TTCP.m[1][3] = tcpPose[1];
    TTCP.m[2][3] = tcpPose[2];

    return T * TTCP;
}

// IK (Inverse Kinematics) Definition
void KinematicSolver::IK(double *pose, bool elbowUp, double *qDeg) {
    // 
    
    // Note: The structure of your IK solver is highly specific 
    // to your robot's Denavit-Hartenberg (DH) parameters.
    
    double px = pose[0], py = pose[1], pz = pose[2];

    double q[DOF] = {0};
    q[0] = atan2(py, px);
    double r = sqrt(px*px + py*py);
    double z = pz - d[0];

    // Assuming the specific structure of the first 3 joints (like a 3-DOF RRR manipulator)
    double L1 = a[1], L2 = d[3];
    double D = (r*r + z*z - L1*L1 - L2*L2)/(2*L1*L2);
    D = constrain(D, -1, 1);

    double q3 = acos(D);
    if (elbowUp) q3 = -q3;

    double q2 = atan2(z, r) - atan2(L2*sin(q3), L1 + L2*cos(q3));

    q[1] = q2; q[2] = q3;

    // Remaining joints (4, 5, 6) likely determined by wrist orientation
    double Rbf[3][3];
    eulerXYZ(pose[3]*DEG_TO_RAD, pose[4]*DEG_TO_RAD, pose[5]*DEG_TO_RAD, Rbf);

    double e[3];
    rotToEulerXYZ(Rbf, e);
    q[3] = e[0]; q[4] = e[1]; q[5] = e[2];

    // Convert radians back to degrees for the output array
    for (int i = 0; i < DOF; i++) qDeg[i] = q[i] * RAD_TO_DEG;
}