#include "robotis_mini/hardware.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotis_mini
{

hardware::CallbackReturn hardware::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    if (hardware_info.joints.empty()) {
        RCLCPP_FATAL(
            this->get_logger(),
            "No joints specified in hardware info"
        );
        return CallbackReturn::ERROR;
    }

    // Store joint names and allocate vectors
    joint_names_.reserve(hardware_info.joints.size());
    for (const auto & joint : hardware_info.joints) {
        joint_names_.push_back(joint.name);
    }

    size_t n = joint_names_.size();
    joint_position_.assign(n, 0.0);
    joint_velocity_.assign(n, 0.0);
    joint_effort_.assign(n, 0.0);

    joint_position_command_.assign(n, 0.0);
    joint_velocity_command_.assign(n, 0.0);
    joint_effort_command_.assign(n, 0.0);

    // TODO initialize hardware

    RCLCPP_INFO(
        rclcpp::get_logger("obotis_mini::hardware"),
        "Hardware interface initialized for %zu joints", n
    );
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
hardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_names_[i],
                hardware_interface::HW_IF_POSITION,
                &joint_position_[i]
            )
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_names_[i],
                hardware_interface::HW_IF_VELOCITY,
                &joint_velocity_[i]
            )
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_names_[i],
                hardware_interface::HW_IF_EFFORT,
                &joint_effort_[i]
            )
        );
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
hardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint_names_[i],
                hardware_interface::HW_IF_POSITION,
                &joint_position_command_[i]
            )
        );
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint_names_[i],
                hardware_interface::HW_IF_VELOCITY,
                &joint_velocity_command_[i]
            )
        );
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint_names_[i],
                hardware_interface::HW_IF_EFFORT,
                &joint_effort_command_[i]
            )
        );
    }
    return command_interfaces;
}

hardware_interface::return_type
hardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // TODO poll hardware
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
hardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // TODO send commands
    }
    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robotis_mini::hardware,
    hardware_interface::SystemInterface
)
