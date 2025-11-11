#include "robotis_mini/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robotis_mini/ik.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace robotis_mini {

kinematics::kinematics(const rclcpp::NodeOptions & options)
: Node("kinematics", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing kinematics");

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&kinematics::joint_state_callback, this, _1));

    ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "actual_end_effector_pose", 10);

    timer_ = this->create_wall_timer(
        publish_period_, std::bind(&kinematics::publish_end_effector_pose, this));

    srv_compute_ik_ = this->create_service<ComputeIK>(
        "compute_ik",
        std::bind(&kinematics::handle_compute_ik, this, _1, _2));

    srv_get_joint_names_ = this->create_service<GetJointNames>(
        "get_joint_names",
        std::bind(&kinematics::handle_get_joint_names, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "kinematics ready");
}

void kinematics::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_state_ = msg;
    RCLCPP_DEBUG(
        this->get_logger(),
        "Received joint state with %zu positions", msg->position.size()
    );
}

void kinematics::publish_end_effector_pose()
{
    if (!current_joint_state_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "No joint state yet; skipping EE pose publish"
        );
        return;
    }

    // TODO: plug in FK (KDL) here. For now publish identity pose.
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "Body"; // use your base link (URDF has "Body")
    pose_msg.pose.orientation.w = 1.0;
    ee_pose_pub_->publish(pose_msg);
}

void kinematics::fill_joint_names(sensor_msgs::msg::JointState & js)
{
    static const char* names[] = {
        "l_shoulder_joint", "l_biceps_joint", "l_elbow_joint",
        "r_shoulder_joint", "r_biceps_joint", "r_elbow_joint",
        "l_hip_joint", "l_thigh_joint", "l_knee_joint", "l_ankle_joint", "l_foot_joint",
        "r_hip_joint", "r_thigh_joint", "r_knee_joint", "r_ankle_joint", "r_foot_joint"
    };
    js.name.assign(std::begin(names), std::end(names));
    js.position.assign(16, 0.0);
    js.velocity.clear();
    js.effort.clear();
}

void kinematics::handle_compute_ik(
    const std::shared_ptr<ComputeIK::Request> req,
    std::shared_ptr<ComputeIK::Response> res)
{
    // positions vector holds DXL ticks for 16 joints; we’ll convert to radians
    std::vector<uint16_t> ticks(16, 512);  // neutral
    // Arms
    if (req->use_arms) {
        ik::IK_RH(req->rh_x, req->rh_y, req->rh_z, ticks);
        ik::IK_LH(req->lh_x, req->lh_y, req->lh_z, ticks);
    }
    // Legs (note: feet IK needs base roll/pitch)
    if (req->use_legs) {
        ik::IK_RF(req->rf_x, req->rf_y, req->rf_z, req->base_roll, req->base_pitch, ticks);
        ik::IK_LF(req->lf_x, req->lf_y, req->lf_z, req->base_roll, req->base_pitch, ticks);
    }

    // Build JointState in radians (ROS controllers consume radians)
    sensor_msgs::msg::JointState js;
    fill_joint_names(js);
    // Map: your IK stores joints in 0..15 order that matches Joint_01..Joint_16 indices:
    // positions[0] -> Joint_01, ..., positions[15] -> Joint_16 (Joint_09 is fixed).
    for (size_t i = 0; i < 16; ++i) {
        js.position[i] = ticks_to_rad(ticks[i]); // (ticks - 512)/195.3786
    }
    js.header.stamp = this->now();

    res->joints = std::move(js);
    res->message = "IK computed from provided legacy model (ticks->rad converted)";
}


void kinematics::handle_get_joint_names(
    const std::shared_ptr<GetJointNames::Request>,
    std::shared_ptr<GetJointNames::Response> res)
{
  sensor_msgs::msg::JointState js;
  fill_joint_names(js);
  res->names = js.name;
}

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotis_mini::kinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
