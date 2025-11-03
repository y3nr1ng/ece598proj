#pragma once
#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

namespace robotis_mini {

class restore_initial_pose : public rclcpp::Node {
public:
    using Action = robotis_mini::action::RestoreInitialPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    explicit restore_initial_pose(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

private:
    rclcpp_action::Server<Action>::SharedPtr server_;

    using FJT = control_msgs::action::FollowJointTrajectory;
    rclcpp_action::Client<FJT>::SharedPtr fjt_client_;

    std::vector<std::string> joint_names_;
    std::vector<double>      initial_positions_;
    std::string              controller_ns_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
                                            std::shared_ptr<const Action::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    void execute_goal(const std::shared_ptr<GoalHandle> goal_handle);
};

}
