#include "robotis_mini/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

// ======== Declarations for legacy IK functions ========
namespace {

// Utility clamp
static inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

// ---- Example constants (replace with your legacy values if different) ----
static constexpr float PI = 3.14159265358979323846f;

// The output vector "positions" must be size 16 (Joint_01..Joint_16)
// Right Hand IK -> writes arm joints on the right side
void IK_RH(float x, float y, float z, std::vector<uint16_t> &positions)
{
  // *** REPLACE THIS BODY with your exact RH IK ***
  // Placeholder: keep neutral
  (void)x; (void)y; (void)z;
  // positions[index] = tick_value;
}

// Left Hand IK
void IK_LH(float x, float y, float z, std::vector<uint16_t> &positions)
{
  // *** REPLACE THIS BODY with your exact LH IK ***
  (void)x; (void)y; (void)z;
}

// Right Foot IK (needs base roll/pitch)
void IK_RF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions)
{
  // *** REPLACE THIS BODY with your exact RF IK ***
  (void)x; (void)y; (void)z; (void)th_r; (void)th_p;
}

// Left Foot IK (needs base roll/pitch)
void IK_LF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions)
{
  // *** REPLACE THIS BODY with your exact LF IK ***
  (void)x; (void)y; (void)z; (void)th_r; (void)th_p;
}

}
// ==================================================================

using std::placeholders::_1;

namespace robotis_mini {

kinematics::kinematics(const rclcpp::NodeOptions & options)
: Node("kinematics", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing kinematics");

  // sub: joint_states (optional for future FK publishing)
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&kinematics::joint_state_callback, this, _1));

  // pub: end-effector pose (placeholder)
  ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mini/actual_end_effector_pose", 10);

  // periodic publisher (placeholder FK)
  timer_ = this->create_wall_timer(
      publish_period_, std::bind(&kinematics::publish_end_effector_pose, this));

  // srv: compute_ik
  srv_compute_ik_ = this->create_service<ComputeIk>(
      "mini/compute_ik",
      std::bind(&kinematics::handle_compute_ik, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "kinematics ready");
}

void kinematics::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_state_ = msg;
  RCLCPP_DEBUG(this->get_logger(), "Received joint state with %zu positions", msg->position.size());
}

void kinematics::publish_end_effector_pose()
{
  if (!current_joint_state_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "No joint state yet; skipping EE pose publish");
    return;
  }

  // TODO: plug in FK (KDL) here. For now publish identity pose.
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "Body"; // use your base link (URDF has "Body")
  pose_msg.pose.orientation.w = 1.0;
  ee_pose_pub_->publish(pose_msg);
}

// Fill Joint_01..Joint_16; Joint_09 is fixed in your URDF, keep it in the list for ordering.
void kinematics::fill_joint_names(sensor_msgs::msg::JointState & js)
{
  static const char* names[] = {
    "Joint_01","Joint_02","Joint_03","Joint_04","Joint_05","Joint_06",
    "Joint_07","Joint_08","Joint_09","Joint_10","Joint_11","Joint_12",
    "Joint_13","Joint_14","Joint_15","Joint_16"
  };
  js.name.assign(std::begin(names), std::end(names));
  js.position.assign(16, 0.0);
  js.velocity.clear();
  js.effort.clear();
}

// ---- IK service handler ------------------------------------------------------
void kinematics::handle_compute_ik(
    const std::shared_ptr<ComputeIk::Request> req,
    std::shared_ptr<ComputeIk::Response> res)
{
  // positions vector holds DXL ticks for 16 joints; we’ll convert to radians
  std::vector<uint16_t> ticks(16, 512);  // neutral
  // Arms
  if (req->use_arms) {
    IK_RH(req->rh_x, req->rh_y, req->rh_z, ticks);
    IK_LH(req->lh_x, req->lh_y, req->lh_z, ticks);
  }
  // Legs (note: feet IK needs base roll/pitch)
  if (req->use_legs) {
    IK_RF(req->rf_x, req->rf_y, req->rf_z, req->base_roll, req->base_pitch, ticks);
    IK_LF(req->lf_x, req->lf_y, req->lf_z, req->base_roll, req->base_pitch, ticks);
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

} // namespace robotis_mini

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotis_mini::kinematics>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
