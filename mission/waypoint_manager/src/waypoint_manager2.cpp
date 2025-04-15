#include <chrono>
#include <deque>
#include <memory>

#include "spdlog/spdlog.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "vortex_msgs/action/reference_filter_waypoint.hpp"
#include "vortex_msgs/srv/waypoint.hpp"

using namespace std::chrono_literals;

class WaypointManager : public rclcpp::Node
{
public:
    using ReferenceFilterWaypoint = vortex_msgs::action::ReferenceFilterWaypoint;
    using GoalHandleRFWP = rclcpp_action::ClientGoalHandle<ReferenceFilterWaypoint>;
    using WaypointService = vortex_msgs::srv::Waypoint;

    WaypointManager() : Node("waypoint_manager"), goal_in_progress_(false)
    {
        service_ = this->create_service<WaypointService>(
            "update_waypoints",
            std::bind(&WaypointManager::waypoint_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        spdlog::info("Waypoint service server started.");

        action_client_ = rclcpp_action::create_client<ReferenceFilterWaypoint>(
            this, "/orca/reference_filter"
        );
        spdlog::info("Waiting for action server to become available...");
        constexpr auto timeout = std::chrono::seconds(10);
        if (!action_client_->wait_for_action_server(timeout)) {
            spdlog::critical("Action server not available after waiting.");
            rclcpp::shutdown();
            return;
        }
        spdlog::info("Action server available.");
    }

private:
    rclcpp_action::Client<ReferenceFilterWaypoint>::SharedPtr action_client_;
    rclcpp::Service<WaypointService>::SharedPtr service_;
    std::deque<geometry_msgs::msg::Point> waypoint_queue_;
    bool goal_in_progress_;

    void waypoint_callback(
        const std::shared_ptr<WaypointService::Request> request,
        std::shared_ptr<WaypointService::Response> response)
    {
        for (auto &pt : request->waypoint) {
            spdlog::info("Adding waypoint: ({}, {}, {})", pt.x, pt.y, pt.z);
            waypoint_queue_.push_back(pt);
        }
        response->success = true;
        if (!goal_in_progress_) {
            send_next_waypoint();
        }
    }

    typename ReferenceFilterWaypoint::Goal waypoint_to_goal(const geometry_msgs::msg::Point & waypoint)
    {
        ReferenceFilterWaypoint::Goal goal_msg;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "some_frame_id";
        pose.header.stamp = this->now();
        pose.pose.position = waypoint;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        goal_msg.goal = pose;
        return goal_msg;
    }

    void send_next_waypoint()
    {
        if (goal_in_progress_) {
            return;
        }
        if (waypoint_queue_.empty()) {
            spdlog::info("No waypoints in queue.");
            return;
        }
        goal_in_progress_ = true;
        auto waypoint = waypoint_queue_.front();
        waypoint_queue_.pop_front();

        const auto goal_msg = waypoint_to_goal(waypoint);
        spdlog::info("Sending waypoint to action server: ({}, {}, {})",
                     waypoint.x, waypoint.y, waypoint.z);

        auto send_goal_options = rclcpp_action::Client<ReferenceFilterWaypoint>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](GoalHandleRFWP::SharedPtr goal_handle) {
                if (!goal_handle) {
                    on_action_server_response(false);
                } 
            };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<ReferenceFilterWaypoint>::WrappedResult & result) {
                on_action_server_response(result.code == rclcpp_action::ResultCode::SUCCEEDED);
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void on_action_server_response(bool success)
    {
        if (!success) {
            spdlog::error("Action failed. Clearing entire waypoint list.");
            waypoint_queue_.clear();
        }
        goal_in_progress_ = false;
        send_next_waypoint();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointManager>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
 