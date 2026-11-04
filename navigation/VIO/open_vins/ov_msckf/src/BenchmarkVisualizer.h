#ifndef OV_MSCKF_BENCHMARK_VISUALIZER_H
#define OV_MSCKF_BENCHMARK_VISUALIZER_H

#include <mutex>

#include "ros/ROS2Visualizer.h"

namespace ov_msckf {

/**
 * @brief Upstream ROS2Visualizer, but the published pose starts at the benchmark odometry pose.
 *
 * out = benchmark_start * (vio_start^-1 * pose)
 * With no `benchmark_topic` in the config, the output simply starts at zero.
 * Uses the output hooks added by cmake/ros2_visualizer_output_hook.patch.
 */
class BenchmarkVisualizer : public ROS2Visualizer {

public:
  using ROS2Visualizer::ROS2Visualizer;

  /// Upstream subscribers plus the optional benchmark odometry
  void setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser);

protected:
  bool transform_output_pose(Eigen::Vector4d &q_xyzw, Eigen::Vector3d &p) override;
  void transform_output_cov(Eigen::Matrix<double, 6, 6> &cov_posori) override;

private:
  /// Saves the offset the first time VIO is initialized and the benchmark pose is known
  bool set_offset_if_needed();

  std::string benchmark_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_benchmark_;
  std::mutex mtx_;

  /// Latest benchmark pose
  bool have_bench_ = false;
  Eigen::Matrix3d R_bench_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_bench_ = Eigen::Vector3d::Zero();

  /// Offset from the OpenVINS global frame to the output frame
  bool have_offset_ = false;
  Eigen::Matrix3d R_off_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_off_ = Eigen::Vector3d::Zero();
};

} // namespace ov_msckf

#endif // OV_MSCKF_BENCHMARK_VISUALIZER_H
