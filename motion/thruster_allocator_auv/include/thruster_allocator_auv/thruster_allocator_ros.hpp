/**
 * @file thruster_allocator_ros.hpp
 * @brief Contains the ROS logic for the thruster allocator node.
 */

#ifndef VORTEX_ALLOCATOR_ROS_HPP
#define VORTEX_ALLOCATOR_ROS_HPP

#include "thruster_allocator_auv/eigen_vector6d_typedef.hpp"
#include "thruster_allocator_auv/pseudoinverse_allocator.hpp"
#include "thruster_allocator_auv/thruster_allocator_utils.hpp"
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class ThrusterAllocator : public rclcpp::Node {
public:
  explicit ThrusterAllocator();

  /**
   * @brief Calculates the allocated
   * thrust based on the body frame forces. It then saturates the output vector
   * between min and max values and publishes the thruster forces to the topic
   * "thrust/thruster_forces".
   */
  void calculate_thrust_timer_cb();

private:
  Eigen::MatrixXd thrust_configuration;

  /**
   * @brief Callback function for the wrench input subscription. Extracts the
   * surge, sway, heave, roll, pitch and yaw values from the received wrench msg
   * and stores them in the body_frame_forces_ Eigen vector.
   * @param msg The received geometry_msgs::msg::Wrench message.
   */
  void wrench_cb(const geometry_msgs::msg::Wrench &msg);

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      thruster_forces_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr
      wrench_subscriber_;
  rclcpp::TimerBase::SharedPtr calculate_thrust_timer_;
  size_t count_;
  int num_dof_;
  int num_thrusters_;
  int min_thrust_;
  int max_thrust_;
  Eigen::Vector6d body_frame_forces_;
  PseudoinverseAllocator pseudoinverse_allocator_;
};

#endif // VORTEX_ALLOCATOR_ROS_HPP
