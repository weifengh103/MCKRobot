#include <iostream>
#include <cmath>
#include <cstring>
#include <iomanip>

#include <math.h>
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

template <size_t R, size_t C>
void printMatrix(const double (&M)[R][C], const char* name = nullptr) {
  static_assert((R == 3 && C == 3) || (R == 4 && C == 4),
                "printMatrix only supports 3x3 or 4x4 matrices");

  if (name)
    std::cout << name << " =\n";

  for (size_t i = 0; i < R; i++) {
    for (size_t j = 0; j < C; j++) {
      std::cout << std::setw(10)
                << std::fixed << std::setprecision(4)
                << M[i][j] << " ";
    }
    std::cout << "\n";
  }
  std::cout << std::endl;
}
/* =========================================================
   Rotation utilities (replaces scipy Rotation)
========================================================= */

// Extrinsic xyz
void eulerExtrinsicXYZ(double rx, double ry, double rz, double R[3][3]) {
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

// Intrinsic XYZ
void rotationMatrixToIntrinsicXYZ(const double R[3][3],
                                  double &rx, double &ry, double &rz) {
  ry = asin(-R[2][0]);

  if (fabs(cos(ry)) > 1e-6) {
    rx = atan2(R[2][1], R[2][2]);
    rz = atan2(R[1][0], R[0][0]);
  } else {
    rx = 0;
    rz = atan2(-R[0][1], R[1][1]);
  }
}

//Extrinsic XYZ
void rotationMatrixToExtrinsicXYZ(const double R[3][3],
                                  double &rx, double &ry, double &rz)
{
    // sy = sin(ry)
    double sy = R[0][2];

    // Clamp for numerical safety
    if (sy >  1.0) sy =  1.0;
    if (sy < -1.0) sy = -1.0;

    ry = asin(sy);

    // cos(ry)
    double cy = cos(ry);

    if (fabs(cy) > 1e-6) {
        rx = atan2(-R[1][2], R[2][2]);
        rz = atan2(-R[0][1], R[0][0]);
    } else {
        // Gimbal lock (ry ≈ ±90°)
        rx = atan2(R[2][1], R[1][1]);
        rz = 0.0;
    }
}

/* =========================================================
   4x4 matrix helpers
========================================================= */

void matMul4(const double A[4][4], const double B[4][4], double C[4][4]) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      C[i][j] = 0;
      for (int k = 0; k < 4; k++)
        C[i][j] += A[i][k] * B[k][j];
    }
}

void matMul3(const double A[3][3], const double B[3][3], double C[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      C[i][j] = 0;
      for (int k = 0; k < 3; k++)
        C[i][j] += A[i][k] * B[k][j];
    }
}

void matCopy4(const double A[4][4], double B[4][4]) {
  memcpy(B, A, sizeof(double) * 16);
}



bool invert3x3(const double m[3][3], double invOut[3][3])
{
    float det =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if (fabs(det) < 1e-6f) {
        return false; // Singular matrix
    }

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


/* =========================================================
   KinematicSolver (structure preserved)
========================================================= */

class KinematicSolver {
public:
  double a[6], d[6], alphas[6], thetas[6];

  KinematicSolver(double *_a, double *_d, double *_alphas, double *_thetas) {
    memcpy(a, _a, sizeof(a));
    memcpy(d, _d, sizeof(d));
    memcpy(alphas, _alphas, sizeof(alphas));
    memcpy(thetas, _thetas, sizeof(thetas));
  }

  /* ---------- FK section ---------- */

  void getDHTransMatrix(int i, const double angelDeg[6], double T[4][4]) {
    double newTheta = thetas[i] + angelDeg[i] * DEG_TO_RAD;
    double ct = cos(newTheta), st = sin(newTheta);
    double ca = cos(alphas[i]), sa = sin(alphas[i]);

    T[0][0] = ct;   T[0][1] = -st * ca;  T[0][2] = st * sa;   T[0][3] = a[i] * ct;
    T[1][0] = st;   T[1][1] = ct * ca;   T[1][2] = -ct * sa;  T[1][3] = a[i] * st;
    T[2][0] = 0;    T[2][1] = sa;        T[2][2] = ca;       T[2][3] = d[i];
    T[3][0] = 0;    T[3][1] = 0;         T[3][2] = 0;        T[3][3] = 1;
  }

  void updateAllTJointJointTrans(const double angelDeg[6],
                                 double TJJ[6][4][4]) {
    for (int i = 0; i < 6; i++)
    {
      getDHTransMatrix(i, angelDeg, TJJ[i]);
      printMatrix(TJJ[i], "TJJ");
    }
  }

  void updateAllTBaseJointTrans(double TJJ[6][4][4],
                                double TBJ[6][4][4]) {
    for (int i = 0; i < 6; i++) {
      if (i == 0)
        matCopy4(TJJ[i], TBJ[i]);
      else
        matMul4(TBJ[i - 1], TJJ[i], TBJ[i]);

      printMatrix(TBJ[i], "TBJ");
    }
  }

  /* ---------- Pose / Transform ---------- */

  void PoseToTransformationMatrix(const double pose[6],
                                  bool isExtrinsic,
                                  double T[4][4]) {
    double R[3][3];
    eulerExtrinsicXYZ(
      pose[3] * DEG_TO_RAD,
      pose[4] * DEG_TO_RAD,
      pose[5] * DEG_TO_RAD,
      R
    );

    memset(T, 0, sizeof(double) * 16);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        T[i][j] = R[i][j];

    T[0][3] = pose[0];
    T[1][3] = pose[1];
    T[2][3] = pose[2];
    T[3][3] = 1;
  }

  void TransformationMatrixToPose(double T[4][4],
                                  bool isExtrinsic,
                                  double pose[6]) {
    pose[0] = T[0][3];
    pose[1] = T[1][3];
    pose[2] = T[2][3];

    double R[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        R[i][j] = T[i][j];

    double rx, ry, rz;
    rotationMatrixToIntrinsicXYZ(R, rx, ry, rz);

    pose[3] = rx * RAD_TO_DEG;
    pose[4] = ry * RAD_TO_DEG;
    pose[5] = rz * RAD_TO_DEG;
  }

  /* ---------- IK ---------- */

  void SolveIK(const double TCPPose[6], bool elbowUp, double outDeg[6]) {
    double angelsRad[6] = {0};

    double px = TCPPose[0];
    double py = TCPPose[1];
    double pz = TCPPose[2];

    angelsRad[0] = atan2(py, px);
    double pxRotated = hypot(px, py);
    if (fabs(angelsRad[0]) > M_PI / 2) pxRotated = -pxRotated;

    double pzLocal = pz - d[0];

    double j3Abs = M_PI - acos(
      (a[1]*a[1] + d[3]*d[3] - pxRotated*pxRotated - pzLocal*pzLocal) /
      (2 * a[1] * d[3])
    );

    if (elbowUp) {
      angelsRad[2] = -j3Abs;
      angelsRad[1] = atan2(pzLocal, pxRotated) +
                     atan2(d[3] * sin(j3Abs),
                           a[1] + d[3] * cos(j3Abs));
    } else {
      angelsRad[2] = j3Abs;
      angelsRad[1] = atan2(pzLocal, pxRotated) -
                     atan2(d[3] * sin(j3Abs),
                           a[1] + d[3] * cos(j3Abs));
    }

    double rmBaseToFlange[3][3];
    eulerExtrinsicXYZ(
      TCPPose[3] * DEG_TO_RAD,
      TCPPose[4] * DEG_TO_RAD,
      TCPPose[5] * DEG_TO_RAD,
      rmBaseToFlange
    );

    double angelsDeg[6];
    for (int i = 0; i < 6; i++)
      angelsDeg[i] = angelsRad[i] * RAD_TO_DEG;

    double TJJ[6][4][4], TBJ[6][4][4];
    updateAllTJointJointTrans(angelsDeg, TJJ);
    updateAllTBaseJointTrans(TJJ, TBJ);

    printMatrix(TBJ[5], "TBJ5");
    double R03SphereEnd[3][3];
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        R03SphereEnd[r][c] = TBJ[5][r][c];

    printMatrix(R03SphereEnd, "R03SphereEnd");

    double rmLink3SphereEndToBase[3][3];
     
    invert3x3(R03SphereEnd, rmLink3SphereEndToBase);

    printMatrix(rmLink3SphereEndToBase, "rmLink3SphereEndToBase");

    double rbLink3EndToFlange [3][3];
    matMul3(rmLink3SphereEndToBase, rmBaseToFlange, rbLink3EndToFlange);

    printMatrix(rbLink3EndToFlange, "rbLink3EndToFlange");
    rotationMatrixToExtrinsicXYZ(
      rbLink3EndToFlange,
      angelsRad[3],
      angelsRad[4],
      angelsRad[5]
    );



    for (int i = 0; i < 6; i++)
      outDeg[i] = angelsRad[i] * RAD_TO_DEG;
  }
};

/* =========================================================
   Console debug main()
========================================================= */

int main() {
  double a[6]      = {0, 50, 0, 0, 0, 0};
  double d[6]      = {50, 0, 0, 50, 0, 0};
  double alphas[6] = {M_PI/2, 0, M_PI/2, M_PI/2, M_PI/2, 0};
  double thetas[6] = {0, 0, M_PI/2, -M_PI, M_PI/2, 0};

  KinematicSolver solver(a, d, alphas, thetas);

  double tcp[6] = {50,10,50,90,0,0};
  // double tcp[6] = {50,0,50,180,0,0};
  double joints[6];

  solver.SolveIK(tcp, true, joints);

  std::cout << "IK solution:\n";
  for (int i = 0; i < 6; i++)
    std::cout << "J" << i + 1 << ": " << joints[i] << " deg\n";

  return 0;
}
