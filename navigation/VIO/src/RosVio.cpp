#include "../include/RosVio.h"

#include "core/VioManager.h"
#include "state/State.h"
#include "utils/print.h"

using namespace ov_msckf;

namespace {
// JPL q_GtoI has the same xyzw as Hamilton q_ItoG
Eigen::Matrix3d rot_from_xyzw(const Eigen::Vector4d &q) { return Eigen::Quaterniond(q(3), q(0), q(1), q(2)).normalized().toRotationMatrix(); }
} // namespace

void RosVio::setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser) {
  ROS2Visualizer::setup_subscribers(parser);

  parser->parse_config("benchmark_topic", benchmark_topic_, false);
  if (benchmark_topic_.empty())
    return;
  sub_benchmark_ = _node->create_subscription<nav_msgs::msg::Odometry>(
      benchmark_topic_, rclcpp::QoS(10), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto &o = msg->pose.pose.orientation;
        const auto &p = msg->pose.pose.position;
        std::lock_guard<std::mutex> lck(mtx_);
        rot_bench_ = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized().toRotationMatrix();
        pos_bench_ << p.x, p.y, p.z;
        have_bench_ = true;
      });
  PRINT_INFO("subscribing to benchmark odometry: %s\n", benchmark_topic_.c_str());
}

bool RosVio::set_offset_if_needed() {
  std::lock_guard<std::mutex> lck(mtx_);
  if (have_offset_)
    return true;
  if (!_app->initialized())
    return false;

  // Wait for the benchmark odometry so both start at the same moment
  if (!benchmark_topic_.empty() && !have_bench_) {
    static bool warned = false;
    if (!warned)
      PRINT_WARNING(YELLOW "VIO initialized, waiting for benchmark odometry on %s\n" RESET, benchmark_topic_.c_str());
    warned = true;
    return false;
  }

  // offset = benchmark_start * vio_start^-1
  std::shared_ptr<State> state = _app->get_state();
  Eigen::Matrix3d rot_start = rot_from_xyzw(state->_imu->quat());
  rot_off_ = rot_bench_ * rot_start.transpose();
  pos_off_ = pos_bench_ - rot_off_ * state->_imu->pos();
  have_offset_ = true;
  PRINT_INFO("Output starts at benchmark pose: p = %.3f %.3f %.3f\n", pos_bench_(0), pos_bench_(1), pos_bench_(2));
  return true;
}

bool RosVio::transform_output_pose(Eigen::Vector4d &q_xyzw, Eigen::Vector3d &p) {
  if (!set_offset_if_needed())
    return false;
  Eigen::Quaterniond q_out(rot_off_ * rot_from_xyzw(q_xyzw));
  q_out.normalize();
  q_xyzw << q_out.x(), q_out.y(), q_out.z(), q_out.w();
  p = rot_off_ * p + pos_off_;
  return true;
}

void RosVio::transform_output_cov(Eigen::Matrix<double, 6, 6> &cov_posori) {
  // Orientation error is in the local frame, so only the position block rotates
  Eigen::Matrix<double, 6, 6> T = Eigen::Matrix<double, 6, 6>::Identity();
  T.block(0, 0, 3, 3) = rot_off_;
  cov_posori = T * cov_posori * T.transpose();
}
