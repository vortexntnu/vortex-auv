#ifndef ESKF_ROS_HPP
#define ESKF_ROS_HPP

#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "eskf/eskf.hpp"
#include "eskf/typedefs.hpp"
#include "spdlog/spdlog.h"

/**
 * @brief ROS2 wrapper for the Error-State Kalman Filter.
 *
 * Subscribes to IMU, DVL, and landmark topics, loads parameters from
 * the YAML config, and publishes filtered odometry. Provides services
 * for toggling NIS gating and forcing anchor resets at runtime.
 */
class ESKFNode : public rclcpp::Node {
   public:
    static constexpr uint16_t INVALID_ID = 0xFFFF;

    explicit ESKFNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    /**
     * @brief Callback for IMU messages
     *
     * @param msg Imu message containing accelerometer and gyroscope data
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief Callback for DVL messages
     *
     * @param msg TwistWithCovarianceStamped containing body-frame velocity
     */
    void dvl_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Callback for landmark detections
     *
     * Selects lowest-ID marker, looks up base<-cam TF, builds a
     * VisualMeasurement, and forwards it to ESKF::landmark_update.
     *
     * @param msg Array of detected ArUco landmarks
     */
    void landmark_callback(const vortex_msgs::msg::LandmarkArray::SharedPtr msg);

    /**
     * @brief Publish filtered odometry from the current nominal state
     */
    void publish_odom();

    void set_subscribers_and_publisher();
    void set_parameters();
    void setup_services();

    /**
     * @brief Subscribes to IMU data
     */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    /**
     * @brief Subscribes to DVL twist data
     */
    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
    /**
     * @brief Subscribes to detected ArUco landmarks
     */
    rclcpp::Subscription<
        vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_sub_;

    /**
     * @brief Publishes filtered odometry
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_pub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_vo_rejects_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_vo_anchor_valid_;

    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    std::unique_ptr<ESKF> eskf_;

    Eigen::Matrix3d R_imu_eskf_{};  /**< Rotation from IMU frame to ESKF frame */
    rclcpp::Time last_imu_time_{};
    bool first_imu_msg_received_;

    static inline void pub_f64(
      const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& pub,
      double v) {
        if (!pub) return;
        std_msgs::msg::Float64 m;
        m.data = v;
        pub->publish(m);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_gating_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_anchor_srv_;

    bool use_vo_;
    std::string vo_base_frame_;
    std::string vo_cam_frame_;
    uint16_t last_marker_id_;
    bool have_last_marker_;
};

#endif  // ESKF_ROS_HPP
