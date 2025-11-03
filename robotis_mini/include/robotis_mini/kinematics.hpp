#pragma once

#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "robotis_mini/srv/compute_ik.hpp"

namespace robotis_mini {

class kinematics : public rclcpp::Node {
public:
    explicit kinematics(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // ---- subscriptions / publishers (kept from your original) ----
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_end_effector_pose(); // FK TODO: placeholder

    // ---- IK service ----
    using ComputeIk = robotis_mini::srv::ComputeIK;
    rclcpp::Service<ComputeIk>::SharedPtr srv_compute_ik_;
    void handle_compute_ik(
        const std::shared_ptr<ComputeIk::Request> req,
        std::shared_ptr<ComputeIk::Response> res);

    // helpers
    static double ticks_to_rad(uint16_t ticks) { return (static_cast<int>(ticks) - 512) / 195.3786; }
    static uint16_t rad_to_ticks(double rad)   { return static_cast<uint16_t>(512 + rad * 195.3786); }

    // Builds JointState with Joint_01..Joint_16 (skips fixed Joint_09 by leaving its value 0)
    static void fill_joint_names(sensor_msgs::msg::JointState & js);

    // incoming / periodic
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds publish_period_{100};
};

}
