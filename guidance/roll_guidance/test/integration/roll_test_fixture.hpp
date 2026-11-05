#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <vector>
#include <cmath>

#include <Eigen/Geometry>
#include <std_msgs/msg/bool.hpp>

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

    /**
     * @brief Sets up the ROLL guidance node and its test environment.
     */
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
            rclcpp::Parameter("topics.killswitch", "/test/killswitch"),
            rclcpp::Parameter("roll.pose_timeout_seconds", pose_timeout()),
            rclcpp::Parameter("roll.trigger_timeout_seconds", trigger_timeout()),
            rclcpp::Parameter("roll.maneuver_timeout_seconds", maneuver_timeout()),
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

        killswitch_pub_ = client_node_->create_publisher<std_msgs::msg::Bool>(
                "/test/killswitch",
                rclcpp::QoS(10).reliable());

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
                gate_pub_->get_subscription_count() > 0 &&
                killswitch_pub_->get_subscription_count() > 0;
        }));
    }

    /**
     * @brief Tears down the ROLL guidance node after each test.
     */
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

    /**
     * @brief Returns the pose freshness timeout for integration tests.
     * @return Timeout in seconds.
     */
    virtual double pose_timeout() const {
        return 2.0;
    }

    /**
     * @brief Returns the gate trigger timeout for integration tests.
     * @return Timeout in seconds.
     */
    virtual double trigger_timeout() const {
        return 3.0;
    }

    /**
     * @brief Returns the maneuver execution timeout for integration tests.
     * @return Timeout in seconds.
     */
    virtual double maneuver_timeout() const {
        return 3.0;
    }

    /**
     * @brief Publishes the killswitch state.
     *
     * @param engaged True to engage the killswitch,
     *                false to release it.
     */
    void publish_killswitch(bool engaged) {
        std_msgs::msg::Bool msg;
        msg.data = engaged;
        killswitch_pub_->publish(msg);
    }

    /**
     * @brief Wait until a condition is true or a timeout occurs.
     */
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

    /**
     * @brief Publishes a gate position.
     */
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

    /**
     * @brief Publishes a pose with covariance.
     */
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

    /**
     * @brief Points to the ROLL guidance node.
     */
    std::shared_ptr<ROLLGuidanceNode> guidance_;

    /**
     * @brief Points to the ROLL guidance action client node.
     */
    rclcpp::Node::SharedPtr client_node_;

    /**
     * @brief Points to the mock Waypoint Manager node.
     */
    rclcpp::Node::SharedPtr manager_node_;

    /**
     * @brief Points to the mock Waypoint MAnager.
     */ 
    std::unique_ptr<MockWaypointManager> manager_;

    /**
     * @brief Points to the ROLL guidance action client.
     */
    rclcpp_action::Client<Action>::SharedPtr client_;

    /**
     * @brief Publishes the pose to the ROLL guidance node.
     */
    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_pub_;

    /**
     * @brief Publishes the gate position to the ROLL guidance node.
     */
    rclcpp::Publisher<
        geometry_msgs::msg::PointStamped>::SharedPtr
        gate_pub_;

    /**
     * @brief Executes the ROS nodes in a separate thread.
     * This allows the test to publish messages and send action goals.
     */
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spin_thread_;

    /**
     * @brief Publishes the killswitch state to the ROLL guidance node.
     */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr killswitch_pub_;

    /**
     * @brief Returns the gate normal vector for integration tests.
     */
    virtual std::vector<double> gate_normal() const {
        return {1.0, 0.0, 0.0};
    }
};

}  // namespace vortex::guidance::test