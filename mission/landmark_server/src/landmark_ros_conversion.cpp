#include <cmath>
#include <vortex/utils/ros/ros_conversions.hpp>
#include "landmark_server/landmark_server_ros.hpp"

namespace vortex::mission {

geometry_msgs::msg::PoseWithCovariance track_to_pose_with_covariance(
    const vortex::filtering::Track& track) {
    geometry_msgs::msg::PoseWithCovariance msg;

    msg.pose.position.x = track.nominal_state.pos(0);
    msg.pose.position.y = track.nominal_state.pos(1);
    msg.pose.position.z = track.nominal_state.pos(2);

    msg.pose.orientation.w = track.nominal_state.ori.w();
    msg.pose.orientation.x = track.nominal_state.ori.x();
    msg.pose.orientation.y = track.nominal_state.ori.y();
    msg.pose.orientation.z = track.nominal_state.ori.z();

    // Position and orientation blocks; no cross terms between them.
    msg.covariance.fill(0.0);
    const auto& P = track.error_state.cov();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            msg.covariance[r * 6 + c] = P(r, c);
            msg.covariance[(r + 3) * 6 + (c + 3)] = P(r + 3, c + 3);
        }
    }
    return msg;
}

namespace {

bool is_valid_landmark_msg(const vortex_msgs::msg::Landmark& lm) {
    const auto& p = lm.pose.pose.position;
    const auto& q = lm.pose.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        return false;
    }
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
        !std::isfinite(q.w)) {
        return false;
    }
    if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-12) {
        return false;
    }
    // Covariance may be all zeros (unknown), but must not contain NaN or
    // negative variances on the diagonal.
    for (size_t i = 0; i < lm.pose.covariance.size(); ++i) {
        const double c = lm.pose.covariance[i];
        if (!std::isfinite(c)) {
            return false;
        }
        if (i % 7 == 0 && c < 0.0) {
            return false;
        }
    }
    return true;
}

}  // namespace

std::vector<Landmark> LandmarkServerNode::ros_msg_to_landmarks(
    const vortex_msgs::msg::LandmarkArray& msg) const {
    std::vector<Landmark> out;
    out.reserve(msg.landmarks.size());
    const double stamp_sec = rclcpp::Time(msg.header.stamp).seconds();

    std::optional<geometry_msgs::msg::Point> vehicle;
    {
        std::lock_guard<std::mutex> lock(odom_mtx_);
        vehicle = last_odom_position_;
    }
    // A copy: the intake rules can change live (on_parameters_set).
    IntakeConfig intake;
    {
        std::lock_guard<std::mutex> lock(intake_mtx_);
        intake = intake_config_;
    }

    for (const auto& lm_msg : msg.landmarks) {
        if (!is_valid_landmark_msg(lm_msg)) {
            ++dropped_measurements_;
            continue;
        }
        Landmark lm;
        lm.pose =
            vortex::utils::ros_conversions::ros_pose_to_pose(lm_msg.pose.pose);
        lm.class_key = vortex::filtering::LandmarkClassKey{
            lm_msg.type.value, lm_msg.subtype.value};
        lm.stamp_sec = stamp_sec;
        if (vehicle && intake.noise.enabled()) {
            // The detector noise along and across the line of sight, on top
            // of the class noise: far detections weigh less, and their depth
            // least.
            const auto& p = lm_msg.pose.pose.position;
            lm.extra_position_cov = intake.noise.covariance(Eigen::Vector3d(
                p.x - vehicle->x, p.y - vehicle->y, p.z - vehicle->z));
        }
        const auto& cov = lm_msg.pose.covariance;
        if (intake.use_measurement_covariance && cov[0] > 0.0 &&
            cov[7] > 0.0 && cov[14] > 0.0) {
            Eigen::Matrix3d pc;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    pc(r, c) = cov[r * 6 + c];
                }
            }
            const double min_std = intake.covariance_min_std_m;
            pc = 0.5 * (pc + pc.transpose()).eval() *
                     intake.covariance_scale +
                 Eigen::Matrix3d::Identity() * min_std * min_std;
            lm.position_cov = pc;
        }
        // A rotational variance >= the limit means "no orientation".
        const double no_ori = intake.no_orientation_rot_variance;
        lm.has_orientation =
            !(cov[3 * 6 + 3] >= no_ori || cov[4 * 6 + 4] >= no_ori ||
              cov[5 * 6 + 5] >= no_ori);
        out.push_back(lm);
    }
    return out;
}

}  // namespace vortex::mission
