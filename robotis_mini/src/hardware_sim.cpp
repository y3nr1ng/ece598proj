#define USE_DYNAMIXEL 0

#include "robotis_mini/hardware.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace robotis_mini {

class hardware_sim : public hardware_impl {};

}

PLUGINLIB_EXPORT_CLASS(
    robotis_mini::hardware_sim,
    hardware_interface::SystemInterface
)