#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "robotis_mini/ik.hpp"

namespace robotis_mini::ik {

// ---------- constants (geometry & unit conversion) ----------
inline constexpr float kPi = 3.14159265358979323846f;

// arm link lengths (mm)
inline constexpr float kLSh = 0.039f;  // origin to arm roll joint
inline constexpr float kLA1 = 0.018f;  // shoulder bracket horizontal
inline constexpr float kLA2 = 0.012f;  // shoulder bracket vertical
inline constexpr float kLA3 = 0.045f;  // upper arm
inline constexpr float kLA4 = 0.072f;  // lower arm

// leg link lengths (mm)
inline constexpr float kLBy   = 0.024f;  // origin to pelvis vertical
inline constexpr float kLBz   = 0.072f;  // pelvis horizontal
inline constexpr float kLBx   = 0.015f;  // shoulder axis to leg center (sagittal)
inline constexpr float kLL1   = 0.006f;   // pelvis roll to pitch offset
inline constexpr float kLL2   = 0.045f;  // thigh
inline constexpr float kLL3   = 0.042f;  // shank
inline constexpr float kLL4   = 0.031f;  // ankle
inline constexpr float kLFoot = 0.009f; // foot horizontal

// elbow reach guard (mm) — matches prior 117 clamp
inline constexpr float kArmReachMax = 0.117f;

// ---------- IK: Right Hand ----------
void IK_RH(float x, float y, float z, std::vector<float> &pos)
{
  // translate user coords to shoulder frame
  const float x0 = x;
  const float y0 = y + (kLSh + kLA1);
  const float z0 = z;

  const float th1 = -std::atan(x0 / (z0 + 1.0e-5f)); // shoulder roll

  // distances in the rotated shoulder plane
  const float xr = x0 - kLA2 * std::sin(th1);
  const float zr = z0 + kLA2 * std::cos(th1);

  const float R1 = std::sqrt(xr * xr + y0 * y0 + zr * zr);
  const float R1c = std::clamp(R1, 0.0f, kArmReachMax);  // enforce reachable envelope

  const float alpha = std::acos((kLA3*kLA3 + kLA4*kLA4 - R1c*R1c) / (2.0f*kLA3*kLA4));
  const float th5 = -kPi + alpha;                    // elbow

  const float R2 = std::hypot(xr, zr);
  float th3;                                         // shoulder pitch
  if (z > 0) {
        th3 = kPi*0.5f + (std::atan2(y0, R2) + std::acos((kLA3*kLA3 + R1c*R1c - kLA4*kLA4) / (2.0f*kLA3*R1c)));
  } else {
        th3 = -kPi*0.5f + (-std::atan2(y0, R2) + std::acos((kLA3*kLA3 + R1c*R1c - kLA4*kLA4) / (2.0f*kLA3*R1c)));
  }

  pos[0] = static_cast<float>(th1); // Joint_01
  pos[1] = static_cast<float>(th3); // Joint_03
  pos[2] = static_cast<float>(th5); // Joint_05
}

// ---------- IK: Left Hand ----------
void IK_LH(float x, float y, float z, std::vector<float> &pos)
{
  const float x0 = x;
  const float y0 = y - (kLSh + kLA1);
  const float z0 = z;

  const float th2 =  std::atan(x0 / (z0 + 1.0e-5f)); // shoulder roll

  const float xr = x0 - kLA2 * std::sin(th2);
  const float zr = z0 + kLA2 * std::cos(th2);

  const float R1 = std::sqrt(xr * xr + y0 * y0 + zr * zr);
  const float R1c = std::clamp(R1, 0.0f, kArmReachMax);

  const float alpha = std::acos((kLA3*kLA3 + kLA4*kLA4 - R1c*R1c) / (2.0f*kLA3*kLA4));
  const float th6 =  kPi - alpha;                    // elbow

  const float R2 = std::hypot(xr, zr);
  float th4;                                         // shoulder pitch
  if (z0 > 0) {
        th4 = -(std::atan2(R2, y0) + std::acos((kLA3*kLA3 + R1c*R1c - kLA4*kLA4) / (2.0f*kLA3*R1c)));
  } else {
        th4 = -(-kPi*0.5f + (std::atan2(y0, R2) + std::acos((kLA3*kLA3 + R1c*R1c - kLA4*kLA4) / (2.0f*kLA3*R1c))));
  }

  pos[3] = static_cast<float>(th2); // Joint_02
  pos[4] = static_cast<float>(th4); // Joint_04
  pos[5] = static_cast<float>(th6); // Joint_06
}

// ---------- IK: Right Foot (needs base roll/pitch) ----------
void IK_RF(float x, float y, float z, float th_r, float th_p, std::vector<float> &pos)
{
  // foot position in hip frame (from original derivation)
  const float px = -kLBz - z - kLFoot*std::sin(th_r) - kLL4*std::cos(th_p)*std::cos(th_r);
  const float py =  kLBy + y + kLFoot*std::cos(th_r) - kLL4*std::cos(th_p)*std::sin(th_r);
  const float pz =  x - kLBx + kLL4*std::sin(th_p);

  const float th7 = std::atan2(py, px);                          // hip yaw/roll combined as in legacy
  const float dx = px - kLL1*std::cos(th7);
  const float dy = py - kLL1*std::sin(th7);
  const float R1 = std::sqrt(dx*dx + dy*dy + pz*pz);
  const float alpha = std::acos((kLL2*kLL2 + kLL3*kLL3 - R1*R1) / (2.0f*kLL2*kLL3));

  const float th11 = kPi - alpha;                                // knee
  const float R2 = std::hypot(dx, dy);

  const float th9 = -(std::atan2(pz, R2) + std::acos((kLL2*kLL2 + R1*R1 - kLL3*kLL3) / (2.0f*kLL2*R1)));
  // Original left ankle IK doesn't seem correct.
  //const float th13 = -std::asin(
  //  std::cos(th9 + th11)*std::cos(th_r)*std::cos(th7)*std::sin(th_p)
  //  - std::sin(th9 + th11)*std::cos(th_p)
  //  + std::cos(th9 + th11)*std::sin(th_p)*std::sin(th_r)*std::sin(th7));
  const float th13 = -std::acos(
      std::cos(th9 + th11)*std::cos(th_p)
    - std::sin(th9 + th11)*std::cos(th_r)*std::cos(th7)*std::sin(th_p)
    - std::sin(th9 + th11)*std::sin(th_p)*std::sin(th_r)*std::sin(th7));
  const float th15 = -std::asin(std::sin(th_r - th7) * std::cos(th_p));

  pos[6]  = static_cast<float>(th7);   // Joint_07
  pos[7]  = static_cast<float>(th9);   // Joint_09
  pos[8]  = static_cast<float>(th11);  // Joint_11
  pos[9]  = static_cast<float>(th13);  // Joint_13
  pos[10] = static_cast<float>(th15);  // Joint_15
}

// ---------- IK: Left Foot (needs base roll/pitch) ----------
void IK_LF(float x, float y, float z, float th_r, float th_p, std::vector<float> &pos)
{
  const float px =  kLFoot*std::sin(th_r) - z - kLBz - kLL4*std::cos(th_p)*std::cos(th_r);
  const float py =  y - kLBy - kLFoot*std::cos(th_r) - kLL4*std::cos(th_p)*std::sin(th_r);
  const float pz =  x - kLBx + kLL4*std::sin(th_p);

  const float th8 = std::atan2(py, px);
  const float dx = px - kLL1*std::cos(th8);
  const float dy = py - kLL1*std::sin(th8);
  const float R1 = std::sqrt(dx*dx + dy*dy + pz*pz);
  const float alpha = std::acos((kLL2*kLL2 + kLL3*kLL3 - R1*R1) / (2.0f*kLL2*kLL3));

  const float th12 = kPi - alpha;                               // knee
  const float R2 = std::hypot(dx, dy);

  const float th10 = -(std::atan2(pz, R2) + std::acos((kLL2*kLL2 + R1*R1 - kLL3*kLL3) / (2.0f*kLL2*R1)));
  const float th14 = -std::acos(
    std::cos(th10 + th12)*std::cos(th_p)
    - std::sin(th10 + th12)*std::cos(th_r)*std::cos(th8)*std::sin(th_p)
    - std::sin(th10 + th12)*std::sin(th_p)*std::sin(th_r)*std::sin(th8));
  const float th16 = -std::asin(std::sin(th_r - th8) * std::cos(th_p));

  pos[11] = static_cast<float>(th8);   // Joint_08
  pos[12] = static_cast<float>(th10);  // Joint_10
  pos[13] = static_cast<float>(th12);  // Joint_12
  pos[14] = static_cast<float>(th14);  // Joint_14
  pos[15] = static_cast<float>(th16);  // Joint_16
}

}
