#include "robotis_mini/servers/restore_initial_pose.hpp"

using namespace std::chrono_literals;

namespace robotis_mini {

restore_initial_pose::restore_initial_pose(const rclcpp::NodeOptions & opts)
    : Node("restore_initial_pose", opts)
{
  declare_parameter<std::vector<std::string>>("joint_names", {});
  declare_parameter<std::vector<double>>("initial_positions", {});
  declare_parameter<std::string>("controller_ns", "/joint_trajectory_controller");

  get_parameter("joint_names", joint_names_);
  get_parameter("initial_positions", initial_positions_);
  get_parameter("controller_ns", controller_ns_);

  if (joint_names_.empty() || initial_positions_.size() != joint_names_.size()) {
    RCLCPP_FATAL(get_logger(), "Params invalid: joint_names or initial_positions mismatch.");
    throw std::runtime_error("restore_initial_pose params invalid");
  }

  // Action server
  server_ = rclcpp_action::create_server<Action>(
      this, "restore_initial_pose",
      std::bind(&restore_initial_pose::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&restore_initial_pose::handle_cancel, this, std::placeholders::_1),
      std::bind(&restore_initial_pose::handle_accepted, this, std::placeholders::_1));

  // FollowJointTrajectory client
  fjt_client_ = rclcpp_action::create_client<FJT>(
      this, controller_ns_ + "/follow_joint_trajectory");
}

rclcpp_action::GoalResponse
restore_initial_pose::handle_goal(const rclcpp_action::GoalUUID &,
                                         std::shared_ptr<const Restore::Goal> goal)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
restore_initial_pose::handle_cancel(const std::shared_ptr<GoalHandle>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void restore_initial_pose::handle_accepted(const std::shared_ptr<GoalHandle> gh)
{
    std::thread{std::bind(&restore_initial_pose::execute_goal, this, gh)}.detach();
}

void restore_initial_pose::execute_goal(const std::shared_ptr<GoalHandle> gh)
{
  const auto goal = gh->get_goal();
  auto feedback = std::make_shared<Restore::Feedback>();
  auto result   = std::make_shared<Restore::Result>();

  // Wait for controller
  if (!fjt_client_->wait_for_action_server(2s)) {
    result->success = false;
    result->message = "Trajectory controller action server not available";
    gh->abort(result);
    return;
  }

  // Build trajectory: single waypoint (home) at t = duration_sec
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joint_names_;
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = initial_positions_;
  pt.time_from_start = rclcpp::Duration::from_seconds(
      goal->duration_sec > 0.0 ? goal->duration_sec : 2.0);

  traj.points.push_back(pt);

  // Send FJT goal
  FJT::Goal fjt_goal;
  fjt_goal.trajectory = traj;
  // optional tolerances can be set here; controller enforces tolerances on success. :contentReference[oaicite:2]{index=2}

  auto send_opts = rclcpp_action::Client<FJT>::SendGoalOptions();
  send_opts.feedback_callback =
      [this, gh, feedback](FJT::GoalHandle::SharedPtr, const std::shared_ptr<const FJT::Feedback> fb) {
        // Best-effort progress estimation from time
        if (fb && fb->desired.time_from_start.nanosec > 0) {
          feedback->progress_percent = 50.0f; // (placeholder; can compute from time)
          gh->publish_feedback(feedback);
        }
      };

  auto goal_handle_future = fjt_client_->async_send_goal(fjt_goal, send_opts);
  if (goal_handle_future.wait_for(2s) != std::future_status::ready) {
    result->success = false;
    result->message = "Failed to send FJT goal";
    gh->abort(result);
    return;
  }
  auto fjt_gh = goal_handle_future.get();
  if (!fjt_gh) {
    result->success = false;
    result->message = "FJT goal rejected";
    gh->abort(result);
    return;
  }

  auto result_future = fjt_client_->async_get_result(fjt_gh);
  // Poll while allowing cancel
  while (rclcpp::ok()) {
    if (gh->is_canceling()) {
      fjt_client_->async_cancel_goal(fjt_gh);
      result->success = false;
      result->message = "Canceled";
      gh->canceled(result);
      return;
    }
    if (result_future.wait_for(100ms) == std::future_status::ready) break;
  }

  auto res = result_future.get();
  if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
    result->success = true;
    result->message = "Restored to initial pose";
    gh->succeed(result);
  } else {
    result->success = false;
    result->message = "Controller reported failure";
    gh->abort(result);
  }
}

} // namespace robotis_mini

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotis_mini::restore_initial_pose>());
  rclcpp::shutdown();
  return 0;
}
