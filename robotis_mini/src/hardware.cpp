#include "robotis_mini/hardware.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotis_mini
{

hardware::CallbackReturn hardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    const auto & info = params.hardware_info;

    if (info.joints.empty()) {
        RCLCPP_FATAL(
            this->get_logger(),
            "No joints specified in hardware info."
        );
        return CallbackReturn::ERROR;
    }

    auto & hw_params = info.hardware_parameters;
    port_name_        = hw_params.at("port");
    baud_rate_        = std::stoi(hw_params.at("baud_rate"));
    protocol_version_ = std::stod(hw_params.at("protocol_version"));

    // Store joint names and allocate vectors.
    size_t n = info.joints.size();
    joint_names_.resize(n);
    joint_ids_.resize(n);
    gear_ratios_.resize(n);

    for (size_t i = 0; i < n; ++i) {
        const auto & joint_info = info.joints[i];
        joint_names_[i] = joint_info.name;

        auto & params = joint_info.parameters;
        joint_ids_[i] = static_cast<uint8_t>(std::stoi(params.at("dxl_id")));
        gear_ratios_[i] = std::stod(params.at("gear_ratio"));
    }

    // Allocate state and command vectors.
    joint_position_.assign(n, 0.0);
    joint_velocity_.assign(n, 0.0);
    joint_effort_.assign(n, 0.0);

    joint_position_command_.assign(n, 0.0);
    joint_velocity_command_.assign(n, 0.0);
    joint_effort_command_.assign(n, 0.0);

    comms_ready_ = false;
    RCLCPP_INFO(
        this->get_logger(),
        "Hardware interface initialized for %zu joints.", n
    );
    return CallbackReturn::SUCCESS;
}

hardware::CallbackReturn
hardware::on_configure(const rclcpp_lifecycle::State & /*prev_state*/)
{
    // Initialize DynamixelSDK.
    port_handler_ =
        std::shared_ptr<dynamixel::PortHandler>(
            dynamixel::PortHandler::getPortHandler(port_name_.c_str()
        ));
    packet_handler_ =
        std::shared_ptr<dynamixel::PacketHandler>(
            dynamixel::PacketHandler::getPacketHandler(protocol_version_)
        );

    if (!port_handler_->openPort()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open port %s", port_name_.c_str());
        return CallbackReturn::ERROR;
    }
    if (!port_handler_->setBaudRate(baud_rate_)) {
        RCLCPP_FATAL(this->get_logger(), "Failed to set baud rate %d", baud_rate_);
        port_handler_->closePort();
        return CallbackReturn::ERROR;
    }

    comms_ready_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "Configured DXL on %s @ %d baud (protocol %.1f)",
        port_name_.c_str(), baud_rate_, protocol_version_
    );
    return CallbackReturn::SUCCESS;
}

hardware::CallbackReturn
hardware::on_cleanup(const rclcpp_lifecycle::State & /*prev_state*/)
{
    if (port_handler_) {
        port_handler_->closePort();
    }

    port_handler_.reset();
    packet_handler_.reset();
    comms_ready_ = false;

    RCLCPP_INFO(this->get_logger(), "Hardware cleaned up.");
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
    if (!comms_ready_) {
        return hardware_interface::return_type::ERROR;
    }

    (void)time;
    (void)period;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        uint8_t id = joint_ids_[i];
        uint32_t dxl_present_pos = 0;
        int comm_result =
            packet_handler_->read4ByteTxRx(
                port_handler_.get(),
                id,
                132, // ADDR_PRESENT_POSITION
                &dxl_present_pos,
                nullptr
            );
        if (comm_result == COMM_SUCCESS) {
            joint_position_[i] =
                convert_dxl_to_rad(dxl_present_pos, gear_ratios_[i]);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Dynamixel read failed for ID %u", id);
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
hardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (!comms_ready_) {
        return hardware_interface::return_type::ERROR;
    }

    (void)time;
    (void)period;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        uint8_t id = joint_ids_[i];
        double cmd_rad = joint_position_command_[i];
        uint32_t dxl_goal_pos = convert_rad_to_dxl(cmd_rad, gear_ratios_[i]);
        int comm_result =
            packet_handler_->write4ByteTxRx(
                port_handler_.get(),
                id,
                116, // ADDR_GOAL_POSITION
                dxl_goal_pos,
                nullptr
            );
        if (comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Dynamixel write failed for ID %u", id);
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

uint32_t hardware::convert_rad_to_dxl(double rad, double gear_ratio) const
{
    double revolutions = rad / (2.0 * M_PI);
    uint32_t dxl = static_cast<uint32_t>(revolutions * 4096.0 * gear_ratio);
    return dxl;
}

double hardware::convert_dxl_to_rad(uint32_t dxl_val, double gear_ratio) const
{
    double revolutions = dxl_val / (4096.0 * gear_ratio);
    double rad = revolutions * (2.0 * M_PI);
    return rad;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robotis_mini::hardware,
    hardware_interface::SystemInterface
)
