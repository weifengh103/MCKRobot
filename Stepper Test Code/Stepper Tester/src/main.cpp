#include <Arduino.h>
#include "KinematicSolver.h"

float a[6]      = {0,50,0,0,0,0};
float d[6]      = {50,0,0,50,0,0};
float alpha[6]  = {M_PI/2, 0, M_PI/2, M_PI/2, M_PI/2, 0};
float theta0[6] = {0, 0, M_PI/2, -M_PI, M_PI/2, 0};



// #include "KinematicSolver.h"
#include <Arduino.h>
// =====================
// Constructor
// =====================

void  printMat3(const Mat3& R) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(R.m[i][j], 6);  // 6 decimal places
      Serial.print("\t");
    }
    Serial.println();
  }
}

 
Mat3 eulerXYZExtrinsicToMat(float rx, float ry, float rz) {
  float cx = cos(rx), sx = sin(rx);
  float cy = cos(ry), sy = sin(ry);
  float cz = cos(rz), sz = sin(rz);

  Mat3 R;
  // Row 0
  R.m[0][0] = cy * cz;
  R.m[0][1] = sx * sy * cz - cx * sz;
  R.m[0][2] = cx * sy * cz + sx * sz;

  // Row 1
  R.m[1][0] = cy * sz;
  R.m[1][1] = sx * sy * sz + cx * cz;
  R.m[1][2] = cx * sy * sz - sx * cz;

  // Row 2
  R.m[2][0] = -sy;
  R.m[2][1] = sx * cy;
  R.m[2][2] = cx * cy;

  return R;
}

// =====================
// Rotation: matrix → intrinsic XYZ
// =====================
Vec3 matToEulerXYZIntrinsic(const Mat3& R) {
  Vec3 e;

  if (fabs(R.m[0][2]) < 1.0f) {
    e.y = asin(R.m[0][2]);
    e.x = atan2(-R.m[1][2], R.m[2][2]);
    e.z = atan2(-R.m[0][1], R.m[0][0]);
  } else {
    // Gimbal lock
    e.y = (R.m[0][2] > 0) ? M_PI / 2 : -M_PI / 2;
    e.x = atan2(R.m[1][0], R.m[1][1]);
    e.z = 0.0f;
  }

  return e;
}

// =====================
// Matrix helpers
// =====================
Mat3 mat3Mul(const Mat3& A, const Mat3& B) {
  Mat3 R = {};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        R.m[i][j] += A.m[i][k] * B.m[k][j];
      }
    }
  }
  return R;
}

Mat3 mat3Transpose(const Mat3& A) {
  Mat3 R;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R.m[i][j] = A.m[j][i];
    }
  }
  return R;
}

// =====================
// DH rotation only
// =====================
Mat3 DHRotation(int i, float jointAngleDeg) {
  float th = theta0[i] + DEG2RAD(jointAngleDeg);
  float ca = cos(alpha[i]), sa = sin(alpha[i]);
  float ct = cos(th), st = sin(th);

  Mat3 R;
  R.m[0][0] = ct;
  R.m[0][1] = -st * ca;
  R.m[0][2] = st * sa;

  R.m[1][0] = st;
  R.m[1][1] = ct * ca;
  R.m[1][2] = -ct * sa;

  R.m[2][0] = 0.0f;
  R.m[2][1] = sa;
  R.m[2][2] = ca;

  return R;
}

// =====================
// Inverse Kinematics
// =====================
void SolveIK(const float tcpPose[6],
                              bool elbowUp,
                              float outAnglesDeg[6]) {
  float px = tcpPose[0];
  float py = tcpPose[1];
  float pz = tcpPose[2];

  float angRad[6] = {0};

  // ---- Joint 1 ----
  angRad[0] = atan2(py, px);

  float pxRot = sqrt(px * px + py * py);
  if (fabs(angRad[0]) > M_PI / 2) pxRot = -pxRot;

  float pzLocal = pz - d[0];

  // ---- Joint 3 ----
  float c3 = (a[1]*a[1] + d[3]*d[3]
             - pxRot*pxRot - pzLocal*pzLocal)
             / (2.0f * a[1] * d[3]);

  if (c3 > 1.0f) c3 = 1.0f;
  if (c3 < -1.0f) c3 = -1.0f;

  float j3abs = M_PI - acos(c3);

  // ---- Joint 2 & 3 ----
  if (elbowUp) {
    angRad[2] = -j3abs;
    angRad[1] = atan2(pzLocal, pxRot)
              + atan2(d[3]*sin(j3abs),
                      a[1] + d[3]*cos(j3abs));
  } else {
    angRad[2] = j3abs;
    angRad[1] = atan2(pzLocal, pxRot)
              - atan2(d[3]*sin(j3abs),
                      a[1] + d[3]*cos(j3abs));
  }

  // ---- TCP rotation (extrinsic xyz) ----
  Mat3 R_base_flange =
    eulerXYZExtrinsicToMat(
      DEG2RAD(tcpPose[3]),
      DEG2RAD(tcpPose[4]),
      DEG2RAD(tcpPose[5])
    );



  // ---- FK rotation up to joint 3 ----
  Mat3 R03;
  bool first = true;

  for (int i = 0; i < 3; i++) {
    Mat3 Ri = DHRotation(i, RAD2DEG(angRad[i]));
    R03 = first ? Ri : mat3Mul(R03, Ri);
    first = false;
  }
  Serial.println("R03-----");
  printMat3(R03);

  Mat3 R30 = mat3Transpose(R03);
  Serial.println("R30-----");
  printMat3(R30);

  Mat3 R36 = mat3Mul(R30, R_base_flange);
  Serial.println("R36-----");
  printMat3(R36);

  // ---- Wrist joints (intrinsic XYZ) ----
  Vec3 wrist = matToEulerXYZIntrinsic(R36);

  angRad[3] = wrist.x;
  angRad[4] = wrist.y;
  angRad[5] = wrist.z;

  for (int i = 0; i < 6; i++) {
    outAnglesDeg[i] = RAD2DEG(angRad[i]);
  }
}






void setup() {
  Serial.begin(115200);
}

void loop() {
  float tcpPose[6] = {50,0,50,180,0,0};
  float joints[6];

  SolveIK(tcpPose, true, joints);

  // for (int i = 0; i < 6; i++) {
  //   Serial.print("J");
  //   Serial.print(i + 1);
  //   Serial.print(": ");
  //   Serial.println(joints[i]);
  // }

  delay(1000);
}
