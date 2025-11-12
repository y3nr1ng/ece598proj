#include <chrono>
#include <map>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "robotis_mini/action/execute_pose.hpp"
#include "robotis_mini/srv/compute_ik.hpp"
#include "robotis_mini/srv/get_joint_names.hpp"

using ExecutePose = robotis_mini::action::ExecutePose;
using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecutePose>;

using FJT        = control_msgs::action::FollowJointTrajectory;
using FJTClient  = rclcpp_action::Client<FJT>;
using FJTHandle  = rclcpp_action::ClientGoalHandle<FJT>;

using ComputeIK  = robotis_mini::srv::ComputeIK;

using GetJointNames = robotis_mini::srv::GetJointNames;

using namespace std::chrono_literals;

class pose : public rclcpp::Node
{
public:
  pose()
  : Node("pose")
  {
    names_client_ = this->create_client<GetJointNames>("get_joint_names");
    ik_client_ = this->create_client<ComputeIK>("compute_ik");
    fjt_client_ = rclcpp_action::create_client<FJT>(
      this, "/joint_trajectory_controller/follow_joint_trajectory"
    );

    // One-shot timer to fetch names after the node enters the executor.
    init_timer_ = this->create_wall_timer(
        100ms,
        [this]() { this->fetch_joints(); });

    action_server_ = rclcpp_action::create_server<ExecutePose>(
      this,
      "execute_pose",
      std::bind(&pose::handle_goal,   this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&pose::handle_cancel, this, std::placeholders::_1),
      std::bind(&pose::handle_accept, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Ready to set pose.");
  }

private:
  std::vector<std::string> joints_;

  rclcpp::TimerBase::SharedPtr init_timer_;

  rclcpp::Client<GetJointNames>::SharedPtr names_client_;
  rclcpp::Client<ComputeIK>::SharedPtr ik_client_;
  rclcpp_action::Client<FJT>::SharedPtr fjt_client_;
  rclcpp_action::Server<ExecutePose>::SharedPtr action_server_;

  std::atomic_bool names_inflight_{false};
  bool have_joints_ = false;

  void fetch_joints() {
    if (have_joints_ || names_inflight_) {
      return;
    }

    if (!names_client_->wait_for_service(0s)) {
      RCLCPP_DEBUG(get_logger(), "Waiting for get_joint_names...");
      return;
    }

    names_inflight_ = true;

    auto req = std::make_shared<robotis_mini::srv::GetJointNames::Request>();
    names_client_->async_send_request(
      req,
      [this](rclcpp::Client<robotis_mini::srv::GetJointNames>::SharedFuture f) {
        names_inflight_ = false;
        try {
          auto resp = f.get();
          if (!resp || resp->names.empty()) {
            RCLCPP_WARN(get_logger(), "get_joint_names returned empty; will retry");
            return;
          }
          joints_ = resp->names;
          have_joints_ = true;
          init_timer_->cancel();
          RCLCPP_INFO(get_logger(), "Loaded %zu joint names from kinematics", joints_.size());
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "get_joint_names failed: %s; will retry", e.what());
        }
      }
    );
  }

  // --- Action callbacks ---
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const ExecutePose::Goal> goal)
  {
    if (goal->duration_sec <= 0.0) {
      RCLCPP_WARN(get_logger(), "Duration_sec must be > 0");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accept(const std::shared_ptr<GoalHandle> gh)
  {
    std::thread(&pose::execute, this, gh).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    if (!have_joints_) {
      auto res = std::make_shared<ExecutePose::Result>();
      res->success = false;
      res->message = "Joints not loaded yet";
      gh->abort(res);
      return;
    }

    const auto goal = gh->get_goal();
    const auto base = this->get_node_base_interface();

    // 1) Wait for IK + FJT
    if (!ik_client_->wait_for_service(std::chrono::seconds(3))) {
      auto r = std::make_shared<ExecutePose::Result>();
      r->success = false;
      r->message = "IK service unavailable";
      gh->abort(r);
      return;
    }
    if (!fjt_client_->wait_for_action_server(std::chrono::seconds(5))) {
      auto r = std::make_shared<ExecutePose::Result>();
      r->success = false;
      r->message = "FollowJointTrajectory not available";
      gh->abort(r);
      return;
    }

    // 2) Call IK
    auto req = std::make_shared<ComputeIK::Request>();
    req->use_arms   = goal->use_arms;
    req->use_legs   = goal->use_legs;
    req->rh_x = goal->rh_x; req->rh_y = goal->rh_y; req->rh_z = goal->rh_z;
    req->lh_x = goal->lh_x; req->lh_y = goal->lh_y; req->lh_z = goal->lh_z;
    req->rf_x = goal->rf_x; req->rf_y = goal->rf_y; req->rf_z = goal->rf_z;
    req->lf_x = goal->lf_x; req->lf_y = goal->lf_y; req->lf_z = goal->lf_z;
    req->base_roll  = goal->base_roll;
    req->base_pitch = goal->base_pitch;

    auto fut = ik_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(base, fut) != rclcpp::FutureReturnCode::SUCCESS) {
      auto r = std::make_shared<ExecutePose::Result>();
      r->success = false;
      r->message = "IK call failed";
      gh->abort(r);
      return;
    }
    const auto resp = fut.get();

    // 3) Reorder IK joints to controller order
    std::vector<double> q(joints_.size(), 0.0);
    std::map<std::string, size_t> ik_index;
    for (size_t i = 0; i < resp->joints.name.size(); ++i)
      ik_index[resp->joints.name[i]] = i;

    for (size_t j = 0; j < joints_.size(); ++j) {
      auto it = ik_index.find(joints_[j]);
      if (it == ik_index.end()) {
        RCLCPP_WARN(
          get_logger(),
          "Controller joint '%s' missing in IK result, using 0.0",
          joints_[j].c_str()
        );
        q[j] = 0.0;
      } else {
        q[j] = resp->joints.position[it->second];
      }
    }

    // 4) Build single-point JointTrajectory
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joints_;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = q;
    p.time_from_start = rclcpp::Duration::from_seconds(goal->duration_sec);
    traj.points.push_back(p);

    // 5) Send FJT goal; translate feedback to progress_percent
    FJT::Goal fjt_goal;
    fjt_goal.trajectory = traj;

    // We’ll compute progress using desired.time_from_start / duration
    const double T = goal->duration_sec;

    auto send_opts = FJTClient::SendGoalOptions{};
    send_opts.feedback_callback =
      [this, gh, T](FJTHandle::SharedPtr,
                    const std::shared_ptr<const FJT::Feedback> fb)
      {
        float p = 0.0f;
        if (!fb->desired.time_from_start.sec == 0 || fb->desired.time_from_start.nanosec != 0) {
          const double t =
            rclcpp::Duration(fb->desired.time_from_start).seconds();
          p = static_cast<float>(std::min(1.0, std::max(0.0, t / T)));
        }
        auto fb_ = std::make_shared<ExecutePose::Feedback>();
        fb_->progress = p;
        gh->publish_feedback(fb_);
      };

    auto gh_future = fjt_client_->async_send_goal(fjt_goal, send_opts);
    if (rclcpp::spin_until_future_complete(base, gh_future) != rclcpp::FutureReturnCode::SUCCESS) {
      auto r = std::make_shared<ExecutePose::Result>();
      r->success = false;
      r->message = "Failed to send FJT goal";
      gh->abort(r); return;
    }
    auto fjt_gh = gh_future.get();
    if (!fjt_gh) {
      auto r = std::make_shared<ExecutePose::Result>();
      r->success = false;
      r->message = "FJT goal rejected";
      gh->abort(r); return;
    }

    auto result_future = fjt_client_->async_get_result(fjt_gh);
    if (rclcpp::spin_until_future_complete(base, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
      auto r = std::make_shared<ExecutePose::Result>();
      r->success = false;
      r->message = "Failed to get FJT result";
      gh->abort(r); return;
    }

    auto wrap = result_future.get();
    auto r = std::make_shared<ExecutePose::Result>();
    switch (wrap.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        r->success = true;
        r->message = "Pose reached";
        gh->succeed(r); break;
      case rclcpp_action::ResultCode::ABORTED:
        r->success = false;
        r->message = "FJT aborted";
        gh->abort(r); break;
      case rclcpp_action::ResultCode::CANCELED:
        r->success = false;
        r->message = "FJT canceled";
        gh->canceled(r); break;
      default:
        r->success = false;
        r->message = "FJT unknown result";
        gh->abort(r); break;
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pose>());
  rclcpp::shutdown();
  return 0;
}
