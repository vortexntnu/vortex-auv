#include "tests/test_VC.hpp"
#include <rmw/types.h>
#include <cmath>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include "velocity_controller/utilities.hpp"
#include "vortex_msgs/msg/los_guidance.hpp"

// Denne noden er kun for å teste velocity_controller noden
test_velocity_controller::test_velocity_controller() : Node("test_VC_node") {
    this->declare_parameter<std::string>("topics.guidance.los");
    this->declare_parameter<std::string>("topics.odom");
    this->topic_guidance =
        this->get_parameter("topics.guidance.los").as_string();
    this->topic_odometry = this->get_parameter("topics.odom").as_string();
    rclcpp::QoS pub_QoS(10);
    pub_QoS.keep_last(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    publisher_guidance = this->create_publisher<vortex_msgs::msg::LOSGuidance>(
        topic_guidance, pub_QoS);
    publisher_state = this->create_publisher<vortex_msgs::msg::LOSGuidance>(
        "/state", pub_QoS);

    rclcpp::QoS sub_QoS(10);
    sub_QoS.keep_last(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    subscription_state = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_odometry, sub_QoS,
        std::bind(&test_velocity_controller::odometry_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                     std::bind(&test_velocity_controller::send_reference, this));
    clock_ = this->get_clock();
    RCLCPP_INFO(this->get_logger(), "Test_velocity_controller node has been started");
}

void test_velocity_controller::send_reference() {
    totaltime += 0.2;
    //reference_msg.yaw = std::numbers::pi * sin(totaltime * std::numbers::pi / 9);
    // reference_msg.pitch=0.3*sin(time1*std::numbers::pi/9);
    reference_msg.yaw = 0.4; 
    reference_msg.surge = 0.20;
    reference_msg.pitch = -0.4;  // reference_msg.yaw=0.0; //Surge, pitch, yaw
    publisher_guidance->publish(reference_msg);
}

void test_velocity_controller::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg_ptr) {
    vortex_msgs::msg::LOSGuidance msg;
    angle temp = quaternion_to_euler_angle(
        msg_ptr->pose.pose.orientation.w, msg_ptr->pose.pose.orientation.x,
        msg_ptr->pose.pose.orientation.y, msg_ptr->pose.pose.orientation.z);
    msg.set__pitch(temp.thetat);
    msg.set__yaw(temp.psit);
    msg.set__surge(msg_ptr->twist.twist.linear.x);
    publisher_state->publish(msg);
}
int main(int argc, char const* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<test_velocity_controller>());
    rclcpp::shutdown();
    return 0;
}
