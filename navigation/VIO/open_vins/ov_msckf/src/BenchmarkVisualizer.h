#ifndef OV_MSCKF_BENCHMARK_VISUALIZER_H
#define OV_MSCKF_BENCHMARK_VISUALIZER_H

#include <mutex>

#include "ros/ROS2Visualizer.h"

namespace ov_msckf {

/**
 * @brief Upstream ROS2Visualizer, but published poses are relative to where VIO started.
 *
 * out = benchmark_start * (vio_start^-1 * pose)
 * If `benchmark_topic` is set in the config, the output starts at that odometry's pose when VIO
 * initializes, so the two can be compared directly. Otherwise the output starts at zero.
 *
 * Needs publish_state() and visualize_odometry() to be virtual in upstream (patched in cmake/ROS2.cmake).
 */
class BenchmarkVisualizer : public ROS2Visualizer {

public:
  using ROS2Visualizer::ROS2Visualizer;

  /// Upstream subscribers plus the optional benchmark odometry
  void setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser);

  void visualize_odometry(double timestamp) override;

protected:
  void publish_state() override;

  /// Store the start pose once VIO is initialized. Returns false while still waiting.
  bool set_start_pose_if_needed();

  /// Map a pose from the OpenVINS global frame to the output frame.
  /// q_xyzw is JPL GtoI (= Hamilton ItoG), p is position in global.
  void to_output_frame(Eigen::Vector4d &q_xyzw, Eigen::Vector3d &p) const;

  /// Rotate a [pos, ori] covariance into the output frame (orientation error is local, so only position changes)
  template <typename Mat> void cov_to_output_frame(Mat &cov) const {
    Eigen::Matrix<double, 6, 6> T = Eigen::Matrix<double, 6, 6>::Identity();
    T.block(0, 0, 3, 3) = R_bench_start_ * R_start_.transpose();
    cov.block(0, 0, 6, 6) = T * cov.block(0, 0, 6, 6) * T.transpose();
  }

  /// Benchmark odometry callback (only keeps the latest pose)
  void callback_benchmark(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// VIO pose (IMU in global) at start
  bool have_start_pose_ = false;
  Eigen::Matrix3d R_start_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_start_ = Eigen::Vector3d::Zero();

  /// Benchmark odometry (empty topic = start at zero)
  std::string benchmark_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_benchmark_;
  bool have_bench_latest_ = false;
  Eigen::Matrix3d R_bench_latest_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_bench_latest_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_bench_start_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_bench_start_ = Eigen::Vector3d::Zero();
  std::mutex start_pose_mtx_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_BENCHMARK_VISUALIZER_H
