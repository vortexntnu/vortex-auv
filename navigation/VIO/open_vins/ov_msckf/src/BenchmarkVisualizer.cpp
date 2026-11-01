#include "BenchmarkVisualizer.h"

#include "core/VioManager.h"
#include "ros/ROSVisualizerHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/print.h"

using namespace ov_msckf;

void BenchmarkVisualizer::setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser) {
  ROS2Visualizer::setup_subscribers(parser);

  parser->parse_config("benchmark_topic", benchmark_topic_, false);
  if (!benchmark_topic_.empty()) {
    sub_benchmark_ = _node->create_subscription<nav_msgs::msg::Odometry>(
        benchmark_topic_, rclcpp::QoS(10), std::bind(&BenchmarkVisualizer::callback_benchmark, this, std::placeholders::_1));
    PRINT_INFO("subscribing to benchmark odometry: %s\n", benchmark_topic_.c_str());
  }
}

void BenchmarkVisualizer::callback_benchmark(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto &o = msg->pose.pose.orientation;
  const auto &t = msg->pose.pose.position;
  std::lock_guard<std::mutex> lck(start_pose_mtx_);
  R_bench_latest_ = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized().toRotationMatrix();
  p_bench_latest_ << t.x, t.y, t.z;
  have_bench_latest_ = true;
}

bool BenchmarkVisualizer::set_start_pose_if_needed() {
  std::lock_guard<std::mutex> lck(start_pose_mtx_);
  if (have_start_pose_)
    return true;
  if (!_app->initialized())
    return false;

  // Wait for the benchmark odometry so both start at the same moment
  if (!benchmark_topic_.empty() && !have_bench_latest_) {
    static bool warned = false;
    if (!warned)
      PRINT_WARNING(YELLOW "VIO initialized, waiting for benchmark odometry on %s\n" RESET, benchmark_topic_.c_str());
    warned = true;
    return false;
  }

  // JPL q_GtoI has the same xyzw as Hamilton q_ItoG
  std::shared_ptr<State> state = _app->get_state();
  Eigen::Vector4d q = state->_imu->quat();
  R_start_ = Eigen::Quaterniond(q(3), q(0), q(1), q(2)).normalized().toRotationMatrix();
  p_start_ = state->_imu->pos();

  // Benchmark pose at the moment VIO starts (identity if no benchmark topic)
  R_bench_start_ = R_bench_latest_;
  p_bench_start_ = p_bench_latest_;

  have_start_pose_ = true;
  PRINT_INFO("Output starts at benchmark pose: p = %.3f %.3f %.3f\n", p_bench_start_(0), p_bench_start_(1), p_bench_start_(2));
  return true;
}

void BenchmarkVisualizer::to_output_frame(Eigen::Vector4d &q_xyzw, Eigen::Vector3d &p) const {
  Eigen::Matrix3d R = Eigen::Quaterniond(q_xyzw(3), q_xyzw(0), q_xyzw(1), q_xyzw(2)).normalized().toRotationMatrix();
  // Relative to VIO start
  Eigen::Matrix3d R_rel = R_start_.transpose() * R;
  Eigen::Vector3d p_rel = R_start_.transpose() * (p - p_start_);
  // Placed on top of the benchmark start pose
  Eigen::Quaterniond q_out(R_bench_start_ * R_rel);
  q_out.normalize();
  q_xyzw << q_out.x(), q_out.y(), q_out.z(), q_out.w();
  p = p_bench_start_ + R_bench_start_ * p_rel;
}

// Same as upstream, but the pose goes through to_output_frame()
void BenchmarkVisualizer::visualize_odometry(double timestamp) {

  // Return if we have not inited and a second has passes
  if (!_app->initialized() || (timestamp - _app->initialized_time()) < 1)
    return;

  // Get fast propagate state at the desired timestamp
  std::shared_ptr<State> state = _app->get_state();
  Eigen::Matrix<double, 13, 1> state_plus = Eigen::Matrix<double, 13, 1>::Zero();
  Eigen::Matrix<double, 12, 12> cov_plus = Eigen::Matrix<double, 12, 12>::Zero();
  if (!_app->get_propagator()->fast_state_propagate(state, timestamp, state_plus, cov_plus))
    return;

  // Express pose relative to the start pose (velocities are in the IMU frame, so they stay as they are)
  if (!set_start_pose_if_needed())
    return;
  {
    Eigen::Vector4d q_rel = state_plus.block(0, 0, 4, 1);
    Eigen::Vector3d p_rel = state_plus.block(4, 0, 3, 1);
    to_output_frame(q_rel, p_rel);
    state_plus.block(0, 0, 4, 1) = q_rel;
    state_plus.block(4, 0, 3, 1) = p_rel;
  }

  // Publish our odometry message if requested
  if (pub_odomimu->get_subscription_count() != 0) {

    nav_msgs::msg::Odometry odomIinM;
    odomIinM.header.stamp = ROSVisualizerHelper::get_time_from_seconds(timestamp);
    odomIinM.header.frame_id = "global";

    // The POSE component (orientation and position)
    odomIinM.pose.pose.orientation.x = state_plus(0);
    odomIinM.pose.pose.orientation.y = state_plus(1);
    odomIinM.pose.pose.orientation.z = state_plus(2);
    odomIinM.pose.pose.orientation.w = state_plus(3);
    odomIinM.pose.pose.position.x = state_plus(4);
    odomIinM.pose.pose.position.y = state_plus(5);
    odomIinM.pose.pose.position.z = state_plus(6);

    // The TWIST component (angular and linear velocities)
    odomIinM.child_frame_id = "imu";
    odomIinM.twist.twist.linear.x = state_plus(7);   // vel in local frame
    odomIinM.twist.twist.linear.y = state_plus(8);   // vel in local frame
    odomIinM.twist.twist.linear.z = state_plus(9);   // vel in local frame
    odomIinM.twist.twist.angular.x = state_plus(10); // we do not estimate this...
    odomIinM.twist.twist.angular.y = state_plus(11); // we do not estimate this...
    odomIinM.twist.twist.angular.z = state_plus(12); // we do not estimate this...

    // Covariance in the order position then orientation as per ros convention
    Eigen::Matrix<double, 12, 12> Phi = Eigen::Matrix<double, 12, 12>::Zero();
    Phi.block(0, 3, 3, 3).setIdentity();
    Phi.block(3, 0, 3, 3).setIdentity();
    Phi.block(6, 6, 6, 6).setIdentity();
    cov_plus = Phi * cov_plus * Phi.transpose();
    for (int r = 0; r < 6; r++) {
      for (int c = 0; c < 6; c++) {
        odomIinM.pose.covariance[6 * r + c] = cov_plus(r, c);
      }
    }
    for (int r = 0; r < 6; r++) {
      for (int c = 0; c < 6; c++) {
        odomIinM.twist.covariance[6 * r + c] = cov_plus(r + 6, c + 6);
      }
    }
    pub_odomimu->publish(odomIinM);
  }

  // Publish our transform on TF
  // NOTE: a rotation from GtoI in JPL has the same xyzw as a ItoG Hamilton rotation
  auto odom_pose = std::make_shared<ov_type::PoseJPL>();
  odom_pose->set_value(state_plus.block(0, 0, 7, 1));
  geometry_msgs::msg::TransformStamped trans = ROSVisualizerHelper::get_stamped_transform_from_pose(_node, odom_pose, false);
  trans.header.stamp = _node->now();
  trans.header.frame_id = "global";
  trans.child_frame_id = "imu";
  if (publish_global2imu_tf) {
    mTfBr->sendTransform(trans);
  }

  // Loop through each camera calibration and publish it
  for (const auto &calib : state->_calib_IMUtoCAM) {
    geometry_msgs::msg::TransformStamped trans_calib = ROSVisualizerHelper::get_stamped_transform_from_pose(_node, calib.second, true);
    trans_calib.header.stamp = _node->now();
    trans_calib.header.frame_id = "imu";
    trans_calib.child_frame_id = "cam" + std::to_string(calib.first);
    if (publish_calibration_tf) {
      mTfBr->sendTransform(trans_calib);
    }
  }
}

// Same as upstream, but the pose and covariance go through to_output_frame()
void BenchmarkVisualizer::publish_state() {

  std::shared_ptr<State> state = _app->get_state();
  if (!set_start_pose_if_needed())
    return;

  // Pose relative to the start pose
  Eigen::Vector4d q_out = state->_imu->quat();
  Eigen::Vector3d p_out = state->_imu->pos();
  to_output_frame(q_out, p_out);

  // We want to publish in the IMU clock frame
  // The timestamp in the state will be the last camera time
  double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
  double timestamp_inI = state->_timestamp + t_ItoC;

  // Create pose of IMU (note we use the bag time)
  geometry_msgs::msg::PoseWithCovarianceStamped poseIinM;
  poseIinM.header.stamp = ROSVisualizerHelper::get_time_from_seconds(timestamp_inI);
  poseIinM.header.frame_id = "global";
  poseIinM.pose.pose.orientation.x = q_out(0);
  poseIinM.pose.pose.orientation.y = q_out(1);
  poseIinM.pose.pose.orientation.z = q_out(2);
  poseIinM.pose.pose.orientation.w = q_out(3);
  poseIinM.pose.pose.position.x = p_out(0);
  poseIinM.pose.pose.position.y = p_out(1);
  poseIinM.pose.pose.position.z = p_out(2);

  // Covariance in the order position then orientation as per ros convention
  std::vector<std::shared_ptr<ov_type::Type>> statevars;
  statevars.push_back(state->_imu->pose()->p());
  statevars.push_back(state->_imu->pose()->q());
  Eigen::Matrix<double, 6, 6> covariance_posori = StateHelper::get_marginal_covariance(_app->get_state(), statevars);
  cov_to_output_frame(covariance_posori);
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      poseIinM.pose.covariance[6 * r + c] = covariance_posori(r, c);
    }
  }
  pub_poseimu->publish(poseIinM);

  // Append to our pose vector and publish the path
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  geometry_msgs::msg::PoseStamped posetemp;
  posetemp.header = poseIinM.header;
  posetemp.pose = poseIinM.pose.pose;
  poses_imu.push_back(posetemp);

  nav_msgs::msg::Path arrIMU;
  arrIMU.header.stamp = _node->now();
  arrIMU.header.frame_id = "global";
  for (size_t i = 0; i < poses_imu.size(); i += std::floor((double)poses_imu.size() / 16384.0) + 1) {
    arrIMU.poses.push_back(poses_imu.at(i));
  }
  pub_pathimu->publish(arrIMU);
}
