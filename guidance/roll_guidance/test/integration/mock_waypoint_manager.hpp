#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <vortex_msgs/action/waypoint_manager.hpp>

namespace vortex::guidance::test {

class MockWaypointManager {
   public:
    using Action = vortex_msgs::action::WaypointManager;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    std::atomic<bool> delay_acceptance{false};
    std::atomic<bool> goal_request_seen{false};
    std::atomic<bool> release_acceptance{false};

    explicit MockWaypointManager(
        const rclcpp::Node::SharedPtr& node,
        const std::string& action_name = "/test/waypoint_manager")
        : node_(node) {

        server_ = rclcpp_action::create_server<Action>(
            node_,
            action_name,
            [this](
                const rclcpp_action::GoalUUID&,
                std::shared_ptr<const Action::Goal>) {

                goal_request_seen = true;

                if (delay_acceptance.load()) {
                    while (!release_acceptance.load()) {
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(5));
                    }
                }

                if (reject_goals.load()) {
                    return rclcpp_action::GoalResponse::REJECT;
                }

                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<GoalHandle>) {
                ++cancel_requests;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> handle) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    active_goal_ = handle;
                }
                ++goals_received;
            });
    }

    /**
     * @brief Completes the active Waypoint Manager goal.
     *
     * @param success Whether the maneuver succeeded.
     * @return true if an active goal was completed.
     */
    bool finish(bool success) {
        std::shared_ptr<GoalHandle> handle;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            handle = active_goal_;
            active_goal_.reset();
        }

        if (!handle || !handle->is_active()) {
            return false;
        }

        auto result = std::make_shared<Action::Result>();
        result->success = success;

        if (success) {
            handle->succeed(result);
        } else {
            handle->abort(result);
        }

        return true;
    }

    std::atomic<bool> reject_goals{false};
    std::atomic<int> goals_received{0};
    std::atomic<int> cancel_requests{0};

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Server<Action>::SharedPtr server_;

    std::mutex mutex_;
    std::shared_ptr<GoalHandle> active_goal_;
};

}  // namespace vortex::guidance::test
