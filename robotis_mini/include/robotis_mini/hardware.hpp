#pragma once

#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"

namespace robotis_mini
{

class hardware : public hardware_interface::SystemInterface
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    hardware() = default;
    ~hardware() override = default;

    CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time,
                                        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time,
                                        const rclcpp::Duration & period) override;

private:
    // Joint names
    std::vector<std::string> joint_names_;

    // State vectors
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    // Command vectors
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_effort_command_;
};

}
