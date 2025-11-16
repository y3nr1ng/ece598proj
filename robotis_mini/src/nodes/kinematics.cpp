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

    // TODO plug in FK, currently it only publish identity pose.
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "Body";
    pose_msg.pose.orientation.w = 1.0;
    ee_pose_pub_->publish(pose_msg);
}

void kinematics::fill_joint_names(sensor_msgs::msg::JointState & js)
{
    static const char* names[] = {
        "r_shoulder_joint", "r_biceps_joint", "r_elbow_joint",
        "l_shoulder_joint", "l_biceps_joint", "l_elbow_joint",
        "r_hip_joint", "r_thigh_joint", "r_knee_joint", "r_ankle_joint", "r_foot_joint",
        "l_hip_joint", "l_thigh_joint", "l_knee_joint", "l_ankle_joint", "l_foot_joint"
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
    std::vector<float> rads(16, 0);  // neutral
    // Arms
    if (req->use_arms) {
        ik::IK_RH(req->rh_x, req->rh_y, req->rh_z, rads);
        ik::IK_LH(req->lh_x, req->lh_y, req->lh_z, rads);
    }
    // Legs
    if (req->use_legs) {
        ik::IK_RF(req->rf_x, req->rf_y, req->rf_z, req->base_roll, req->base_pitch, rads);
        ik::IK_LF(req->lf_x, req->lf_y, req->lf_z, req->base_roll, req->base_pitch, rads);
    }

    // Build JointState in radians.
    sensor_msgs::msg::JointState js;
    fill_joint_names(js);
    for (size_t i = 0; i < 16; ++i) {
        js.position[i] = rads[i];
    }
    js.header.stamp = this->now();

    res->joints = std::move(js);
    res->message = "IK computed";
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
