#pragma once

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"

#ifndef USE_DYNAMIXEL
#define USE_DYNAMIXEL 0
#endif

#if USE_DYNAMIXEL
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/packet_handler.h"
#endif

namespace robotis_mini
{

class hardware_impl : public hardware_interface::SystemInterface
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    hardware_impl() = default;
    ~hardware_impl() override = default;

    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & prev_state) override;
    CallbackReturn on_cleanup  (const rclcpp_lifecycle::State & prev_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time,
                                        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time,
                                        const rclcpp::Duration & period) override;

private:
    // Mapped joints
    std::vector<std::string> joint_names_;
    std::vector<uint8_t>     joint_ids_;

    // State vectors
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    // Command vectors
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_effort_command_;

#if USE_DYNAMIXEL
    // Dynamixel SDK
    std::shared_ptr<dynamixel::PortHandler>   port_handler_;
    std::shared_ptr<dynamixel::PacketHandler> packet_handler_;
#endif
    bool comms_ready_{false};

    std::string port_name_;
    int         baud_rate_{0};
    double      protocol_version_{0.0};

    uint32_t convert_rad_to_dxl(double rad, double gear_ratio = 1.0) const;
    double   convert_dxl_to_rad(uint32_t dxl_val, double gear_ratio = 1.0) const;
};

}
