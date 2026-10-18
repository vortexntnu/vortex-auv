/**
 * @file ukf_ros.hpp
 * @brief Contains the UKFNode class, the ROS 2 interface of the UKF.
 */

#ifndef UKF__ROS__UKF_ROS_HPP_
#define UKF__ROS__UKF_ROS_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include "ukf/filters/ukf.hpp"
#include "ukf/models/measurement_models.hpp"
#include "ukf/models/strapdown_ins.hpp"
#include "ukf/typedefs.hpp"

/**
 * @brief ROS 2 node running the UKF, fusing IMU, DVL, pressure and
 * magnetometer measurements into an odometry estimate.
 */
class UKFNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructor for the UKFNode class.
     * @param options The node options.
     */
    explicit UKFNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    /**
     * @brief Declares the parameters and builds the models and the filter.
     */
    void set_parameters();

    /**
     * @brief Creates the subscribers, publishers and the reset service.
     */
    void set_subscribers_and_publishers();

    /**
     * @brief Looks up the static sensor transforms from TF, retried by a timer
     * until all of them are available.
     */
    void lookup_static_transforms();

    /**
     * @brief Sets the parameters, creates the subscribers and publishers and
     * starts the publish timer. Called once all required TF transforms are
     * available, or immediately if none use TF.
     */
    void complete_initialization();

    /**
     * @brief Resets the estimate to the configured prior.
     */
    void reset_estimate();

    /**
     * @brief Callback for the IMU, runs the prediction step.
     * @param msg The IMU message.
     */
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    /**
     * @brief Callback for the DVL, runs the velocity update and publishes
     * the NIS.
     * @param msg The DVL twist message.
     */
    void dvl_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr
            msg);

    /**
     * @brief Callback for the pressure sensor, runs the depth update and
     * publishes the NIS.
     * @param msg The fluid pressure message.
     */
    void pressure_callback(
        const sensor_msgs::msg::FluidPressure::ConstSharedPtr msg);

    /**
     * @brief Callback for the magnetometer, runs the field update and
     * publishes the NIS.
     * @param msg The magnetic field message.
     */
    void magnetometer_callback(
        const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);

    /**
     * @brief Publishes the current estimate as odometry, and optionally as
     * pose, twist, TF and biases.
     */
    void publish_odom();

    /**
     * @brief Broadcasts the estimate as a TF from odom to base_link.
     * @param state The state to broadcast.
     * @param time The time stamp of the transform.
     */
    void publish_tf(const State& state, const rclcpp::Time& time);

    /**
     * @brief Checks that an aiding measurement is recent, close to the
     * current estimate in time and newer than the previous one.
     * @param stamp The stamp of the measurement.
     * @param previous_stamp The stamp of the previous accepted measurement,
     * updated if this one is accepted.
     * @return True if the measurement should be used.
     */
    bool accept_aiding_stamp(const rclcpp::Time& stamp, double& previous_stamp);

    /**
     * @brief Gates an update on its NIS, and applies it if it passes. Rejected
     * updates leave the estimate unchanged.
     * @tparam M The dimension of the measurement.
     * @param update The output of the filter update.
     * @param threshold The NIS threshold.
     * @param nis The NIS of the update, NaN if it could not be computed.
     * @return True if the update was applied.
     */
    template <int M>
    bool apply_update(const UpdateOutput<M>& update,
                      double threshold,
                      double& nis);

    /**
     * @brief Prefixes a frame name with the frame prefix, if there is one.
     * @param name The frame name.
     * @return The prefixed frame name.
     */
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
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        accel_bias_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        gyro_bias_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_dvl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_depth_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_magnetometer_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        dvl_body_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::TimerBase::SharedPtr odom_pub_timer_;
    std::chrono::milliseconds time_step_{1};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    std::unique_ptr<UnscentedKalmanFilter> ukf_;
    // the dvl and depth models are built per message, their noise can come
    // from the message
    std::unique_ptr<MagnetometerMeasurementModel> magnetometer_model_;

    // the filter itself is stateless, the estimate lives here
    INS3DConfig ins_config_;
    TransformConfig transform_config_{1.0, 2.0, 0.0};
    StateGaussian prior_;
    StateGaussian estimate_;
    ImuInput latest_input_;
    rclcpp::Time last_imu_time_{};
    Eigen::Vector3d previous_gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_gyro_measurement_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d gyro_covariance_ = Eigen::Matrix3d::Zero();

    bool first_imu_msg_received_ = false;
    bool propagated_ = false;
    bool dvl_received_ = false;
    bool depth_received_ = false;
    bool faulted_ = false;
    double last_dvl_stamp_ = -1;
    double last_depth_stamp_ = -1;
    double last_magnetometer_stamp_ = -1;

    std::string frame_prefix_{""};
    double max_imu_dt_ = 0.1;
    double max_estimate_age_ = 0.25;
    double max_aiding_skew_ = 0.05;
    double dvl_nis_threshold_;
    double depth_nis_threshold_;
    double magnetometer_nis_threshold_;

    bool publish_debug_{false};
    bool publish_nis_{false};
    bool publish_tf_{false};
    bool publish_pose_{false};
    bool publish_twist_{false};
    bool publish_biases_{false};

    bool imu_use_tf_transform_{true};
    Eigen::Matrix3d R_imu_ukf_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T_imu_ukf_ = Eigen::Vector3d::Zero();

    bool dvl_use_tf_transform_{true};
    bool dvl_use_msg_noise_{true};
    Eigen::Vector3d dvl_measurement_noise_std_{0.1, 0.1, 0.1};
    Eigen::Matrix3d R_dvl_ukf_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T_dvl_ukf_ = Eigen::Vector3d::Zero();

    // the magnetometer is the one in the Nucleus, so it follows the dvl frame
    bool magnetometer_use_tf_transform_{true};
    Eigen::Matrix3d R_magnetometer_ukf_ = Eigen::Matrix3d::Identity();

    bool pressure_use_tf_transform_{true};
    bool pressure_use_msg_noise_{false};
    double pressure_measurement_noise_{40000.0};
    Eigen::Vector3d T_depth_ukf_ = Eigen::Vector3d::Zero();

    double gravity_;
    double water_density_;
    double atmospheric_pressure_;
    bool pressure_is_gauge_{false};
};

#endif  // UKF__ROS__UKF_ROS_HPP_
