#pragma once

#include <cstdint>
#include <vector>

namespace robotis_mini::ik {

void IK_RH(float, float, float, std::vector<uint16_t>&);
void IK_LH(float, float, float, std::vector<uint16_t>&);
void IK_RF(float, float, float, float roll, float pitch, std::vector<uint16_t>&);
void IK_LF(float, float, float, float roll, float pitch, std::vector<uint16_t>&);

}