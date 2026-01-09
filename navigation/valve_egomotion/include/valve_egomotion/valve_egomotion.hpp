#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace valve_egomotion {

class ValveEgomotionNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructor: Initializes parameters, subscribers, and TF
     * listeners.
     */
    ValveEgomotionNode();

   private:
    std::string ref_frame_;
    bool have_ref_ = false;
    tf2::Transform T_base_marker0_;

    rclcpp::Time last_det_stamp_;
    double lost_timeout_sec_ = 0.5;

    /**
     * @brief Listener. Triggered every time ArUco markers/Valves are detected.
     *
     * @param msg A list of poses where the camera thinks the markers are.
     */
    void valvePoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    /**
     * @brief A memory entry in the sliding window
     */
    struct Sample {
        rclcpp::Time stamp;
        tf2::Transform T_ref_base;
        double weight;
    };

    /**
     * @brief Clean memory for relevant and updated samples in regards to the
     * window size
     *
     * @param nowt Current timestamp of rosbag when used with --clock
     */
    void prune(const rclcpp::Time& nowt);

    /**
     * @brief Averages all samples in the buffer to find one pose.
     *
     * @param T_est The resulting smoothed position and rotation
     * @param cov6x6 An array representing the trust
     * @param nowt Current time for age-checking
     *
     * @return true if estimation succeeded, false if there was insufficient
     * data
     */
    bool estimateWindow(tf2::Transform& T_est,
                        double* cov6x6,
                        const rclcpp::Time& nowt);

    /**
     * @brief SO(3) used so we can average rotation
     */
    static tf2::Vector3 logMapSO3(const tf2::Quaternion& q);

    /**
     * @brief SO(3) wraps a flat 3D vector to valid 3D rotation
     */
    static tf2::Quaternion expMapSO3(const tf2::Vector3& w);

    /**
     * @brief Calculates the angle (radians) between two orientations
     */
    static double angularDist(const tf2::Quaternion& q1,
                              const tf2::Quaternion& q2);

    /**
     * @brief Returns the median of a list
     */
    template <typename T>
    static T getMedian(std::vector<T> v);

    /**
     * @brief YAML config parameters
     */
    std::string map_frame_, base_frame_, cam_frame_;
    int win_size_;
    double win_age_sec_, huber_deg_, gate_deg_, pub_rate_hz_;
    bool use_mad_, publish_tf_;

    /**
     * @brief Memory buffer for sliding window
     */
    std::deque<Sample> buf_;

    /**
     * @brief TF infrastructure
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /**
     * @brief Subscribes to aruco marker/valve pose
     */
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_marker_;

    /**
     * @brief Publishes time stamped pose with covariance
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pub_pose_cov_;

    /**
     * @brief Internal timing
     */
    rclcpp::Time last_pub_time_;
};

}  // namespace valve_egomotion