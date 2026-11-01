#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <vortex_msgs/action/guidance_waypoint.hpp>

#include "roll_guidance/roll_guidance_ros.hpp"
#include "mock_waypoint_manager.hpp"

namespace vortex::guidance::test {

using namespace std::chrono_literals;

class ROLLIntegrationFixture : public ::testing::Test {
   protected:
    using Action = vortex_msgs::action::GuidanceWaypoint;
    using ClientHandle = rclcpp_action::ClientGoalHandle<Action>;

    void SetUp() override {
        rclcpp::NodeOptions options;

        options.parameter_overrides({
            rclcpp::Parameter("topics.pose", "/test/pose"),
            rclcpp::Parameter("topics.waypoint", "/test/gate"),
            rclcpp::Parameter("action_servers.roll", "/test/roll"),
            rclcpp::Parameter(
                "action_servers.waypoint_manager",
                "/test/waypoint_manager"),
            rclcpp::Parameter("roll.trigger_distance", 3.0),
            rclcpp::Parameter("test.disable_shutdown", true),
            rclcpp::Parameter("roll.gate_normal", gate_normal()),
        });

        guidance_ = std::make_shared<ROLLGuidanceNode>(options);
        client_node_ = std::make_shared<rclcpp::Node>("roll_client_test");
        manager_node_ = std::make_shared<rclcpp::Node>("manager_test");

        manager_ =
            std::make_unique<MockWaypointManager>(manager_node_);

        pose_pub_ = client_node_->create_publisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/test/pose", rclcpp::SensorDataQoS());

        gate_pub_ = client_node_->create_publisher<
            geometry_msgs::msg::PointStamped>(
                "/test/gate", rclcpp::QoS(10));

        client_ = rclcpp_action::create_client<Action>(
            client_node_, "/test/roll");

        executor_.add_node(guidance_);
        executor_.add_node(client_node_);
        executor_.add_node(manager_node_);

        spin_thread_ = std::thread([this] {
            executor_.spin();
        });

        ASSERT_TRUE(client_->wait_for_action_server(3s));

        ASSERT_TRUE(wait_until([this] {
            return pose_pub_->get_subscription_count() > 0 &&
                   gate_pub_->get_subscription_count() > 0;
        }));
    }

    void TearDown() override {
        if (manager_) {
            manager_->release_acceptance = true;
        }
        
        executor_.cancel();

        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }

        executor_.remove_node(manager_node_);
        executor_.remove_node(client_node_);
        executor_.remove_node(guidance_);

        manager_.reset();
        client_.reset();
        guidance_.reset();
        manager_node_.reset();
        client_node_.reset();
    }

    bool wait_until(
        const std::function<bool()>& condition,
        std::chrono::milliseconds timeout = 3000ms) {

        const auto deadline =
            std::chrono::steady_clock::now() + timeout;

        while (std::chrono::steady_clock::now() < deadline) {
            if (condition()) {
                return true;
            }
            std::this_thread::sleep_for(10ms);
        }

        return condition();
    }

    void publish_gate(double x = 0.0,
                      double y = 0.0,
                      double z = 0.0) {
        geometry_msgs::msg::PointStamped msg;
        msg.header.frame_id = "map";
        msg.point.x = x;
        msg.point.y = y;
        msg.point.z = z;
        gate_pub_->publish(msg);
    }

    void publish_pose(double x, double y = 0.0,
                      double z = 0.0) {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = z;
        msg.pose.pose.orientation.w = 1.0;
        pose_pub_->publish(msg);
    }

    std::shared_ptr<ClientHandle> start_roll() {
        auto future = client_->async_send_goal(Action::Goal{});

        if (future.wait_for(3s) != std::future_status::ready) {
            return nullptr;
        }

        return future.get();
    }

    std::shared_ptr<ROLLGuidanceNode> guidance_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Node::SharedPtr manager_node_;

    std::unique_ptr<MockWaypointManager> manager_;

    rclcpp_action::Client<Action>::SharedPtr client_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_pub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PointStamped>::SharedPtr
        gate_pub_;

    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spin_thread_;

    virtual std::vector<double> gate_normal() const {
        return {1.0, 0.0, 0.0};
    }
};

}  // namespace vortex::guidance::test