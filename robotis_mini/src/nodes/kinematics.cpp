#include "robotis_mini/kinematics.hpp"

using namespace std::chrono_literals;

namespace robotis_mini {

kinematics::kinematics(const rclcpp::NodeOptions & options)
    : Node("kinematics", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing kinematics");

    joint_state_sub_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&kinematics::joint_state_callback, this, std::placeholders::_1)
        );

    ee_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mini/actual_end_effector_pose",
            10
        );

    timer_ =
        this->create_wall_timer(
            publish_period_,
            std::bind(&kinematics::publish_end_effector_pose, this)
        );

    RCLCPP_INFO(this->get_logger(), "kinematics initialized");
}

void kinematics::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_state_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received joint state with %zu positions", msg->position.size());
}

void kinematics::publish_end_effector_pose()
{
    if (!current_joint_state_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "No joint state received yet; skipping end-effector pose publish"
        );
        return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "base_link";
    // TODO actual kinematics solver call
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;

    ee_pose_pub_->publish(pose_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published end-effector pose");
}

bool kinematics::handle_inverse_kinematics(/* request & response */)
{
  RCLCPP_INFO(this->get_logger(), "Received inverse kinematics request");
  // bool success = kinematics_solver_->inverse(...);
  return true;
}

}  // namespace robotis_mini

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotis_mini::kinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
