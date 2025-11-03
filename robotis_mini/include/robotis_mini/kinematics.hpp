#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robotis_mini {

class kinematics : public rclcpp::Node {
public:
    explicit kinematics(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void publish_end_effector_pose();

    bool handle_inverse_kinematics(/* TODO request/response */);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;

    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds publish_period_{100};
};

}