#ifndef KINEMATIC_SOLVER_H
#define KINEMATIC_SOLVER_H

#include <math.h>

// =====================
// Macros
// =====================
#define DEG2RAD(x) ((x) * M_PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / M_PI)

// =====================
// Basic math structs
// =====================
struct Vec3 {
  float x;
  float y;
  float z;
};

struct Mat3 {
  float m[3][3];
};

// =====================
// KinematicSolver class
// =====================
class KinematicSolver {
public:
  // DH parameters
  float a[6];
  float d[6];
  float alpha[6];
  float theta0[6];

  // Constructor
  KinematicSolver(const float* a_,
                  const float* d_,
                  const float* alpha_,
                  const float* theta0_);

  // -------- IK --------
  // tcpPose = {x, y, z, rx, ry, rz}  (deg for rotations)
  // elbowUp = true / false
  // outAnglesDeg = 6 joint angles in degrees
  void SolveIK(const float tcpPose[6],
               bool elbowUp,
               float outAnglesDeg[6]);

private:
  // -------- Rotation helpers --------
  Mat3 eulerXYZExtrinsicToMat(float rx, float ry, float rz);
  Vec3 matToEulerXYZIntrinsic(const Mat3& R);

  // -------- Matrix helpers --------
  Mat3 mat3Mul(const Mat3& A, const Mat3& B);
  Mat3 mat3Transpose(const Mat3& A);

  // -------- DH --------
  Mat3 DHRotation(int jointIdx, float jointAngleDeg);

  void printMat3(const Mat3& R);
};

#endif  // KINEMATIC_SOLVER_H
