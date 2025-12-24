#include "KinematicSolver.h"

#define K_DEG_TO_RAD (M_PI / 180.0)
#define K_RAD_TO_DEG (180.0 / M_PI)

KinematicSolver::KinematicSolver(const double _a[6], const double _d[6], const double _alphas[6], const double _thetas[6]) {
    memcpy(a, _a, sizeof(a));
    memcpy(d, _d, sizeof(d));
    memcpy(alphas, _alphas, sizeof(alphas));
    memcpy(thetas, _thetas, sizeof(thetas));
}

void KinematicSolver::printMatrix3x3(const double M[3][3], const char* name) {
    if (name) Serial.println(name);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Serial.printf("%10.4f ", M[i][j]);
        }
        Serial.println();
    }
}

void KinematicSolver::printMatrix4x4(const double M[4][4], const char* name) {
    if (name) Serial.println(name);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Serial.printf("%10.4f ", M[i][j]);
        }
        Serial.println();
    }
}

void KinematicSolver::eulerExtrinsicXYZ(double rx, double ry, double rz, double R[3][3]) {
    double cx = cos(rx), sx = sin(rx);
    double cy = cos(ry), sy = sin(ry);
    double cz = cos(rz), sz = sin(rz);
    R[0][0] = cy * cz;
    R[0][1] = cz * sx * sy - cx * sz;
    R[0][2] = sx * sz + cx * cz * sy;
    R[1][0] = cy * sz;
    R[1][1] = cx * cz + sx * sy * sz;
    R[1][2] = cx * sy * sz - cz * sx;
    R[2][0] = -sy;
    R[2][1] = cy * sx;
    R[2][2] = cx * cy;
}

void KinematicSolver::rotationMatrixToIntrinsicXYZ(const double R[3][3], double &rx, double &ry, double &rz) {
    ry = asin(-R[2][0]);
    if (fabs(cos(ry)) > 1e-6) {
        rx = atan2(R[2][1], R[2][2]);
        rz = atan2(R[1][0], R[0][0]);
    } else {
        rx = 0;
        rz = atan2(-R[0][1], R[1][1]);
    }
}

void KinematicSolver::rotationMatrixToExtrinsicXYZ(const double R[3][3], double &rx, double &ry, double &rz) {
    double sy = constrain(R[0][2], -1.0, 1.0);
    ry = asin(sy);
    double cy = cos(ry);
    if (fabs(cy) > 1e-6) {
        rx = atan2(-R[1][2], R[2][2]);
        rz = atan2(-R[0][1], R[0][0]);
    } else {
        rx = atan2(R[2][1], R[1][1]);
        rz = 0.0;
    }
}

void KinematicSolver::matMul4(const double A[4][4], const double B[4][4], double C[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 4; k++) C[i][j] += A[i][k] * B[k][j];
        }
    }
}

void KinematicSolver::matMul3(const double A[3][3], const double B[3][3], double C[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 3; k++) C[i][j] += A[i][k] * B[k][j];
        }
    }
}

void KinematicSolver::matCopy4(const double A[4][4], double B[4][4]) {
    memcpy(B, A, sizeof(double) * 16);
}

bool KinematicSolver::invert3x3(const double m[3][3], double invOut[3][3]) {
    float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (fabs(det) < 1e-6f) return false;
    float invDet = 1.0f / det;
    invOut[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    invOut[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * invDet;
    invOut[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;
    invOut[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * invDet;
    invOut[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    invOut[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * invDet;
    invOut[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    invOut[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * invDet;
    invOut[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;
    return true;
}

void KinematicSolver::getDHTransMatrix(int i, const double angelDeg[6], double T[4][4]) {
    double newTheta = thetas[i] + angelDeg[i] * K_DEG_TO_RAD;
    double ct = cos(newTheta), st = sin(newTheta);
    double ca = cos(alphas[i]), sa = sin(alphas[i]);
    T[0][0] = ct;   T[0][1] = -st * ca;  T[0][2] = st * sa;   T[0][3] = a[i] * ct;
    T[1][0] = st;   T[1][1] = ct * ca;   T[1][2] = -ct * sa;  T[1][3] = a[i] * st;
    T[2][0] = 0;    T[2][1] = sa;        T[2][2] = ca;        T[2][3] = d[i];
    T[3][0] = 0;    T[3][1] = 0;         T[3][2] = 0;         T[3][3] = 1;
}

void KinematicSolver::updateAllTJointJointTrans(const double angelDeg[6], double TJJ[6][4][4]) {
    for (int i = 0; i < 6; i++) {
        getDHTransMatrix(i, angelDeg, TJJ[i]);
    }
}

void KinematicSolver::updateAllTBaseJointTrans(double TJJ[6][4][4], double TBJ[6][4][4]) {
    for (int i = 0; i < 6; i++) {
        if (i == 0) matCopy4(TJJ[i], TBJ[i]);
        else matMul4(TBJ[i - 1], TJJ[i], TBJ[i]);
    }
}

void KinematicSolver::solveIK(const double TCPPose[6], bool elbowUp, double outDeg[6]) {
    double angelsRad[6] = {0};
    double px = TCPPose[0], py = TCPPose[1], pz = TCPPose[2];

    angelsRad[0] = atan2(py, px);
    double pxRotated = hypot(px, py);
    if (fabs(angelsRad[0]) > M_PI / 2) pxRotated = -pxRotated;

    double pzLocal = pz - d[0];
    double j3Abs = M_PI - acos((a[1]*a[1] + d[3]*d[3] - pxRotated*pxRotated - pzLocal*pzLocal) / (2 * a[1] * d[3]));

    if (elbowUp) {
        angelsRad[2] = -j3Abs;
        angelsRad[1] = atan2(pzLocal, pxRotated) + atan2(d[3] * sin(j3Abs), a[1] + d[3] * cos(j3Abs));
    } else {
        angelsRad[2] = j3Abs;
        angelsRad[1] = atan2(pzLocal, pxRotated) - atan2(d[3] * sin(j3Abs), a[1] + d[3] * cos(j3Abs));
    }

    double rmBaseToFlange[3][3];
    eulerExtrinsicXYZ(TCPPose[3] * K_DEG_TO_RAD, TCPPose[4] * K_DEG_TO_RAD, TCPPose[5] * K_DEG_TO_RAD, rmBaseToFlange);

    double angelsDeg[6];
    for (int i = 0; i < 6; i++) angelsDeg[i] = angelsRad[i] * K_RAD_TO_DEG;

    double TJJ[6][4][4], TBJ[6][4][4];
    updateAllTJointJointTrans(angelsDeg, TJJ);
    updateAllTBaseJointTrans(TJJ, TBJ);

    double R03SphereEnd[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R03SphereEnd[r][c] = TBJ[5][r][c];

    double rmLink3SphereEndToBase[3][3];
    invert3x3(R03SphereEnd, rmLink3SphereEndToBase);

    double rbLink3EndToFlange [3][3];
    matMul3(rmLink3SphereEndToBase, rmBaseToFlange, rbLink3EndToFlange);

    rotationMatrixToExtrinsicXYZ(rbLink3EndToFlange, angelsRad[3], angelsRad[4], angelsRad[5]);

    for (int i = 0; i < 6; i++) outDeg[i] = angelsRad[i] * K_RAD_TO_DEG;
}