#ifndef UKF__ROS__UKF_ROS_HPP_
#define UKF__ROS__UKF_ROS_HPP_

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include "ukf/filters/ukf.hpp"
#include "ukf/models/measurement_models.hpp"
#include "ukf/models/strapdown_ins.hpp"
#include "ukf/typedefs.hpp"

class UKFNode : public rclcpp::Node {
   public:
    explicit UKFNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void set_parameters();

    void set_subscribers_and_publishers();

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    void dvl_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr
            msg);

    void pressure_callback(
        const sensor_msgs::msg::FluidPressure::ConstSharedPtr msg);

    void magnetometer_callback(
        const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);

    void publish_odom();

    std::string frame(const std::string& name) const {
        return frame_prefix_.empty() ? name : frame_prefix_ + "/" + name;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr
        pressure_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr
        magnetometer_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_dvl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_depth_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_magnetometer_pub_;
    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    std::unique_ptr<UnscentedKalmanFilter> ukf_;
    std::unique_ptr<DVLMeasurementModel> dvl_model_;
    std::unique_ptr<DepthMeasurementModel> depth_model_;
    std::unique_ptr<MagnetometerMeasurementModel> magnetometer_model_;

    // the filter itself is stateless, the estimate lives here
    StateGaussian estimate_;
    ImuInput latest_input_;
    rclcpp::Time last_imu_time_{};
    bool first_imu_msg_received_ = false;

    std::string frame_prefix_{""};
    double max_imu_dt_ = 0.1;
    double gravity_;
    double water_density_;
    double atmospheric_pressure_;
    bool pressure_is_gauge_{false};
};

#endif  // UKF__ROS__UKF_ROS_HPP_
