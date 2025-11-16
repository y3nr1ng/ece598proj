#pragma once

#include <cstdint>
#include <vector>

namespace robotis_mini::ik {

void IK_RH(float, float, float, std::vector<float>&);
void IK_LH(float, float, float, std::vector<float>&);
void IK_RF(float, float, float, float roll, float pitch, std::vector<float>&);
void IK_LF(float, float, float, float roll, float pitch, std::vector<float>&);

}