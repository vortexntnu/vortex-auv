#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP

#include "thruster_allocator_auv/pseudoinverse_allocator.hpp"
<<<<<<< HEAD
#include "thruster_allocator_auv/eigen_typedefs.hpp"
    =======
#include "thruster_allocator_auv/thruster_allocator_utils.hpp"
    >>>>>>> fd49a378da993c6bc294661bbf715160d6c91b21
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

    class ThrusterAllocator : public rclcpp::Node {
public:
  explicit ThrusterAllocator();

  /**
   * @brief Calculates the allocated
   * thrust based on the body frame forces. It then saturates the output vector
   * between min and max values and publishes the thruster forces to the topic
   * "thrust/thruster_forces".
   */
  void timer_callback();

private:
  Eigen::MatrixXd thrust_configuration;

  /**
   * @brief Callback function for the wrench input subscription. Extracts the
   * surge, sway and yaw values from the received wrench msg
   * and stores them in the body_frame_forces_ Eigen vector.
   * @param msg The received geometry_msgs::msg::Wrench message.
   */
  void wrench_callback(const geometry_msgs::msg::Wrench &msg);

  /**
   * @brief Checks if the given Eigen vector contains any NaN or Inf values
   * @param v The Eigen vector to check.
   * @return True if the vector is healthy, false otherwise.
   */
  bool healthyWrench(const Eigen::VectorXd &v) const;

  rclcpp::Publisher<vortex_msgs::msg::ThrusterForces>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  int num_dof_;
  int num_thrusters_;
  int min_thrust_;
  int max_thrust_;
  Eigen::Vector6d body_frame_forces_;
  std::vector<int64_t> direction_;
  PseudoinverseAllocator pseudoinverse_allocator_;
};

#endif // VORTEX_ALLOCATOR_ALLOCATOR_ROS_HPP
