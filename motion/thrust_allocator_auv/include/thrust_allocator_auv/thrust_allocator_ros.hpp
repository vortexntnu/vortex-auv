/**
 * @file thrust_allocator_ros.hpp
 * @brief Contains the ROS logic for the thruster allocator node.
 */

#ifndef VORTEX_ALLOCATOR_ROS_HPP
#define VORTEX_ALLOCATOR_ROS_HPP

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>
#include "thrust_allocator_auv/eigen_vector6d_typedef.hpp"
#include "thrust_allocator_auv/pseudoinverse_allocator.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"

class ThrustAllocator : public rclcpp::Node {
   public:
    explicit ThrustAllocator(const rclcpp::NodeOptions& options);

    /**
     * @brief Calculates the allocated
     * thrust based on the body frame forces. It then saturates the output
     * vector between min and max values and publishes the thruster forces to
     * the topic "thrust/thruster_forces".
     */
    void calculate_thrust_timer_cb();

   private:
    /**
     * @brief Callback function for the wrench input subscription. Extracts the
     * surge, sway, heave, roll, pitch and yaw values from the received wrench
     * msg and stores them in the body_frame_forces_ Eigen vector.
     * @param msg The received geometry_msgs::msg::Wrench message.
     */
    void wrench_cb(const geometry_msgs::msg::WrenchStamped& msg);

    /**
     * @brief Callback function for the watchdog timer. Checks if the last
     * received message is older than the timeout threshold and publishes zeros
     * to the thruster forces topic if it is.
     */
    void watchdog_callback();

    /**
     * @brief Checks if the given Eigen vector contains any NaN or Inf values
     * @param v The Eigen vector to check.
     * @return True if the vector is healthy, false otherwise.
     */
    bool healthy_wrench(const Eigen::VectorXd& v) const;

    /**
     * @brief Extracts the parameters from the config file.
     */
    void extract_parameters();

    /**
     * @brief Creates the configuration matrix and calculates the pseudoinverse
     * matrix for the thrust allocator.
     */
    void set_allocator();

    /**
     * @brief Sets the wrench subscriber and thrust publisher for the node.
     */
    void set_subscriber_and_publisher();

    rclcpp::Publisher<vortex_msgs::msg::ThrusterForces>::SharedPtr
        thruster_forces_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
        wrench_subscriber_;
    rclcpp::TimerBase::SharedPtr calculate_thrust_timer_;

    size_t count_;
    Eigen::Vector3d center_of_mass_;
    int num_dimensions_;
    int num_thrusters_;
    int min_thrust_;
    int max_thrust_;
    double thrust_update_rate_;

    std::chrono::milliseconds thrust_update_period_;

    Eigen::MatrixXd thruster_force_direction_;
    Eigen::MatrixXd thruster_position_;
    Eigen::MatrixXd thrust_configuration_;

    Eigen::Vector6d body_frame_forces_;
    std::unique_ptr<PseudoinverseAllocator> pseudoinverse_allocator_;

    rclcpp::Time last_msg_time_;
    rclcpp::Duration timeout_treshold_ = std::chrono::seconds(1);
    bool watchdog_triggered_ = false;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

#endif  // VORTEX_ALLOCATOR_ROS_HPP
