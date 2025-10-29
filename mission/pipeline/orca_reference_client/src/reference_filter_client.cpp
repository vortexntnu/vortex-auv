#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "vortex_msgs/action/reference_filter_waypoint.hpp"

using namespace std::chrono_literals;

class ReferenceFilterWaypointClient : public rclcpp::Node {
public:
  using ReferenceFilterWaypoint = vortex_msgs::action::ReferenceFilterWaypoint;
  using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<ReferenceFilterWaypoint>;

  ReferenceFilterWaypointClient()
  : Node("reference_filter_waypoint_client"), current_wp_index_(0)
  {
    action_name_ = declare_parameter<std::string>("action_name", "/orca/reference_filter");
    frame_id_    = declare_parameter<std::string>("frame_id", "base_link");


    waypoints_ = {
      {2.0, 0.0, -1.0},
      {2.0, 2.0, -1.0},
      {0.0, 2.0, -1.0},
      {0.0, 0.0, -1.0}
    };

    client_ = rclcpp_action::create_client<ReferenceFilterWaypoint>(this, action_name_);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&ReferenceFilterWaypointClient::try_connect_and_send, this));
  }

private:
  void try_connect_and_send() {
    if (!client_->wait_for_action_server(0s)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for action server '%s'...", action_name_.c_str());
      return;
    }
    timer_->cancel();
    send_next_goal();
  }

  void send_next_goal() {
    if (current_wp_index_ >= waypoints_.size()) {
      RCLCPP_INFO(get_logger(), "Mission complete! All waypoints reached.");
      rclcpp::shutdown();
      return;
    }

    const auto& wp = waypoints_[current_wp_index_];
    double x = wp[0], y = wp[1], z = wp[2];

    ReferenceFilterWaypoint::Goal goal_msg;
    goal_msg.goal.header.stamp = now();
    goal_msg.goal.header.frame_id = frame_id_;
    goal_msg.goal.pose.position.x = x;
    goal_msg.goal.pose.position.y = y;
    goal_msg.goal.pose.position.z = z;
    goal_msg.goal.pose.orientation.w = 1.0;
    goal_msg.goal.pose.orientation.x = 0.0;
    goal_msg.goal.pose.orientation.y = 0.0;
    goal_msg.goal.pose.orientation.z = 0.0;

    RCLCPP_INFO(get_logger(),
                "Sending waypoint %zu/%zu -> [x=%.2f, y=%.2f, z=%.2f]",
                current_wp_index_ + 1, waypoints_.size(), x, y, z);

    rclcpp_action::Client<ReferenceFilterWaypoint>::SendGoalOptions opts;

    opts.goal_response_callback =
      [this](const GoalHandleWaypoint::SharedPtr handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected by Reference Filter server.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server.");
        }
      };

    opts.feedback_callback =
      [this](GoalHandleWaypoint::SharedPtr /*handle*/,
             const std::shared_ptr<const ReferenceFilterWaypoint::Feedback> feedback) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Feedback: Reference filter running...");
      };

    opts.result_callback =
      [this](const GoalHandleWaypoint::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Waypoint reached successfully.");
        } else {
          RCLCPP_WARN(this->get_logger(), "Waypoint failed (code %d)", result.code);
        }
        current_wp_index_++;
        send_next_goal();
      };

    client_->async_send_goal(goal_msg, opts);
  }

  rclcpp_action::Client<ReferenceFilterWaypoint>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string action_name_, frame_id_;
  std::vector<std::array<double, 3>> waypoints_;
  size_t current_wp_index_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReferenceFilterWaypointClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
