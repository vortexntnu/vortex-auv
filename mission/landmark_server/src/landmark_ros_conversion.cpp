#include <cmath>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
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

    msg.covariance.fill(0.0);

    const auto& Pp = track.error_state.cov();
    msg.covariance[0 * 6 + 0] = Pp(0, 0);  // xx
    msg.covariance[0 * 6 + 1] = Pp(0, 1);  // xy
    msg.covariance[0 * 6 + 2] = Pp(0, 2);  // xz

    msg.covariance[1 * 6 + 0] = Pp(1, 0);  // yx
    msg.covariance[1 * 6 + 1] = Pp(1, 1);  // yy
    msg.covariance[1 * 6 + 2] = Pp(1, 2);  // yz

    msg.covariance[2 * 6 + 0] = Pp(2, 0);  // zx
    msg.covariance[2 * 6 + 1] = Pp(2, 1);  // zy
    msg.covariance[2 * 6 + 2] = Pp(2, 2);  // zz

    msg.covariance[3 * 6 + 3] = Pp(3, 3);  // RR
    msg.covariance[3 * 6 + 4] = Pp(3, 4);  // RP
    msg.covariance[3 * 6 + 5] = Pp(3, 5);  // RY

    msg.covariance[4 * 6 + 3] = Pp(4, 3);  // PR
    msg.covariance[4 * 6 + 4] = Pp(4, 4);  // PP
    msg.covariance[4 * 6 + 5] = Pp(4, 5);  // PY

    msg.covariance[5 * 6 + 3] = Pp(5, 3);  // YR
    msg.covariance[5 * 6 + 4] = Pp(5, 4);  // YP
    msg.covariance[5 * 6 + 5] = Pp(5, 5);  // YY

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

    for (const auto& lm_msg : msg.landmarks) {
        if (!is_valid_landmark_msg(lm_msg)) {
            ++dropped_measurements_;
            continue;
        }
        // Distant pipes are unreliable.
        if (lm_msg.type.value == vortex_msgs::msg::LandmarkType::SLALOM_PIPE &&
            vehicle) {
            const auto& p = lm_msg.pose.pose.position;
            const double d =
                std::hypot(std::hypot(p.x - vehicle->x, p.y - vehicle->y),
                           p.z - vehicle->z);
            if (d > map_config_.intake.max_pipe_distance_m) {
                ++dropped_measurements_;
                continue;
            }
        }
        Landmark lm;
        lm.pose =
            vortex::utils::ros_conversions::ros_pose_to_pose(lm_msg.pose.pose);
        lm.class_key = vortex::filtering::LandmarkClassKey{
            lm_msg.type.value, lm_msg.subtype.value};
        lm.stamp_sec = stamp_sec;
        if (vehicle) {
            const auto& p = lm_msg.pose.pose.position;
            const double d =
                std::hypot(std::hypot(p.x - vehicle->x, p.y - vehicle->y),
                           p.z - vehicle->z);
            const auto& intake = map_config_.intake;
            const double depth_var = intake.noise_base_variance +
                                     intake.noise_variance_per_meter * d;
            lm.extra_variance = depth_var;
            if (d > 1e-6) {
                // depth_var along the line of sight, a fraction across it.
                const Eigen::Vector3d ray =
                    Eigen::Vector3d(p.x - vehicle->x, p.y - vehicle->y,
                                    p.z - vehicle->z) /
                    d;
                const double lateral_var =
                    intake.noise_lateral_ratio * depth_var;
                lm.extra_position_cov =
                    Eigen::Matrix3d::Identity() * lateral_var +
                    (depth_var - lateral_var) * ray * ray.transpose();
            }
        }
        // A rotational variance >= the limit means "no orientation".
        const auto& cov = lm_msg.pose.covariance;
        const double no_ori = map_config_.intake.no_orientation_rot_variance;
        lm.has_orientation =
            !(cov[3 * 6 + 3] >= no_ori || cov[4 * 6 + 4] >= no_ori ||
              cov[5 * 6 + 5] >= no_ori);
        out.push_back(lm);
    }
    return out;
}

vortex_msgs::msg::LandmarkArray LandmarkServerNode::tracks_to_landmark_msgs(
    uint16_t type,
    uint16_t subtype) const {
    vortex_msgs::msg::LandmarkArray out;

    std::vector<const vortex::filtering::Track*> tracks;
    if (subtype == 0) {
        for (const auto& t : track_manager_->get_tracks()) {
            if (t.confirmed && t.class_key.type == type) {
                tracks.push_back(&t);
            }
        }
    } else {
        tracks = track_manager_->get_tracks_by_type(
            vortex::filtering::LandmarkClassKey{type, subtype});
    }
    out.landmarks.reserve(tracks.size());

    const auto stamp = this->now();

    out.header.stamp = stamp;
    out.header.frame_id = target_frame_;

    for (const auto* t : tracks) {
        vortex_msgs::msg::Landmark lm;

        lm.type.value = t->class_key.type;
        lm.subtype.value = t->class_key.subtype;
        lm.id = t->id;

        lm.pose.pose =
            vortex::utils::ros_conversions::to_pose_msg(t->to_pose());

        out.landmarks.push_back(std::move(lm));
    }

    return out;
}

vortex_msgs::msg::Landmark LandmarkServerNode::track_to_landmark_msg(
    const vortex::filtering::Track& track) const {
    vortex_msgs::msg::Landmark lm;
    lm.header.stamp = this->now();
    lm.header.frame_id = target_frame_;
    lm.id = track.id;
    lm.type.value = track.class_key.type;
    lm.subtype.value = track.class_key.subtype;

    lm.pose = track_to_pose_with_covariance(track);

    return lm;
}

}  // namespace vortex::mission
