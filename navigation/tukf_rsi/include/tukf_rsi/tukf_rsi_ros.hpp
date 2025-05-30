#ifndef TUKF_NODE_HPP
#define TUKF_NODE_HPP

#include <memory>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include "tukf_rsi/tukf.hpp"
#include "tukf_rsi/tukf_rsi_utils.hpp"
#include "tukf_rsi/typedefs.hpp"

class TUKFNode : public rclcpp::Node {
public:
    TUKFNode();

private:

    // @brief Callback function for the gyro topic
    // @param msg: Imu message containing the gyro data
    void gyro_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // @brief Callback function for the DVL topic
    // @param msg: TwistWithCovarianceStamped message containing the DVL data
    void dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    // @brief Callback function for the wrench topic
    // @param msg: WrenchStamped message containing the wrench data
    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    
    // @brief Set the subscriber and publisher for the node
    void set_subscribers_and_publisher();

    // @brief Set the parameters for the eskf
    void set_parameters();

    // @brief Publish the odometry message
    void publish_odom();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_sub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::TimerBase::SharedPtr odom_timer_;

    std::unique_ptr<TUKF> tukf_;

    AUVState state_;

    double dt_;

    Eigen::Matrix3d R_gyro_;

    Eigen::Matrix3d R_dvl_;
};

#endif // TUKF_NODE_HPP