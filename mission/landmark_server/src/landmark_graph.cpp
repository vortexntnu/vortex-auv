#include "landmark_server/landmark_graph.hpp"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <numbers>
#include <utility>
#include <vector>

namespace vortex::mission {

namespace {

constexpr double kDegToRad = std::numbers::pi / 180.0;

template <typename T>
T get_or(const YAML::Node& node, const char* key, T fallback) {
    if (node && node[key]) {
        return node[key].as<T>();
    }
    return fallback;
}

double wrap_angle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

gtsam::Pose3 to_pose3(const Eigen::Isometry3d& T) {
    return gtsam::Pose3(gtsam::Rot3(T.rotation()), T.translation());
}

Eigen::Isometry3d to_isometry(const gtsam::Pose3& p) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = p.rotation().matrix();
    T.translation() = p.translation();
    return T;
}

gtsam::Key pose_key(std::size_t index) {
    return gtsam::Symbol('x', index);
}

gtsam::Key landmark_key(int id) {
    return gtsam::Symbol('l', static_cast<std::uint64_t>(id));
}

/**
 * @brief A landmark position measured in the vehicle frame.
 * Error: pose^-1 * point - measured.
 */
class PointInBodyFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3> {
   public:
    PointInBodyFactor(gtsam::Key pose,
                      gtsam::Key point,
                      const gtsam::Point3& measured,
                      const gtsam::SharedNoiseModel& model)
        : NoiseModelFactorN(model, pose, point), measured_(measured) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        const gtsam::Point3& point,
        boost::optional<gtsam::Matrix&> H_pose = boost::none,
        boost::optional<gtsam::Matrix&> H_point = boost::none) const override {
        gtsam::Matrix36 Hp;
        gtsam::Matrix3 Hl;
        const gtsam::Point3 in_body = pose.transformTo(
            point, H_pose ? &Hp : nullptr, H_point ? &Hl : nullptr);
        if (H_pose) {
            *H_pose = Hp;
        }
        if (H_point) {
            *H_point = Hl;
        }
        return in_body - measured_;
    }

   private:
    gtsam::Point3 measured_;
};

/**
 * @brief Absolute roll, pitch and depth (z) of a pose: the parts of the
 * odometry that do not drift. Error: [roll, pitch, z] - measured.
 */
class AttitudeDepthFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
   public:
    AttitudeDepthFactor(gtsam::Key pose,
                        double roll,
                        double pitch,
                        double z,
                        const gtsam::SharedNoiseModel& model)
        : NoiseModelFactorN(model, pose), roll_(roll), pitch_(pitch), z_(z) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        boost::optional<gtsam::Matrix&> H = boost::none) const override {
        gtsam::Matrix36 H_rot;
        gtsam::Matrix36 H_trans;
        gtsam::Matrix13 H_roll;
        gtsam::Matrix13 H_pitch;
        const gtsam::Rot3& R = pose.rotation(H ? &H_rot : nullptr);
        const gtsam::Point3& t = pose.translation(H ? &H_trans : nullptr);
        const double roll = R.roll(H ? &H_roll : nullptr);
        const double pitch = R.pitch(H ? &H_pitch : nullptr);
        if (H) {
            gtsam::Matrix J(3, 6);
            J.row(0) = H_roll * H_rot;
            J.row(1) = H_pitch * H_rot;
            J.row(2) = H_trans.row(2);
            *H = J;
        }
        return gtsam::Vector3(wrap_angle(roll - roll_),
                              wrap_angle(pitch - pitch_), t.z() - z_);
    }

   private:
    double roll_;
    double pitch_;
    double z_;
};

}  // namespace

LandmarkGraphConfig parse_graph_config(const YAML::Node& node) {
    LandmarkGraphConfig c;
    if (!node) {
        return c;
    }
    c.enable = get_or<bool>(node, "enable", c.enable);
    if (const auto kf = node["keyframe"]) {
        c.keyframe_distance_m =
            get_or<double>(kf, "distance_m", c.keyframe_distance_m);
        c.keyframe_angle_deg =
            get_or<double>(kf, "angle_deg", c.keyframe_angle_deg);
        c.keyframe_interval_sec =
            get_or<double>(kf, "interval_sec", c.keyframe_interval_sec);
    }
    if (const auto odom = node["odom_noise"]) {
        c.odom_pos_std_per_m =
            get_or<double>(odom, "pos_std_per_m", c.odom_pos_std_per_m);
        c.odom_yaw_std_deg_per_m =
            get_or<double>(odom, "yaw_std_deg_per_m", c.odom_yaw_std_deg_per_m);
        c.odom_yaw_std_deg_per_sec = get_or<double>(odom, "yaw_std_deg_per_sec",
                                                    c.odom_yaw_std_deg_per_sec);
        c.odom_min_pos_std_m =
            get_or<double>(odom, "min_pos_std_m", c.odom_min_pos_std_m);
        c.odom_min_rot_std_deg =
            get_or<double>(odom, "min_rot_std_deg", c.odom_min_rot_std_deg);
    }
    if (const auto abs = node["absolute"]) {
        c.roll_pitch_std_deg =
            get_or<double>(abs, "roll_pitch_std_deg", c.roll_pitch_std_deg);
        c.depth_std_m = get_or<double>(abs, "depth_std_m", c.depth_std_m);
    }
    if (const auto m = node["measurements"]) {
        c.huber_k = get_or<double>(m, "huber_k", c.huber_k);
        c.max_measurements_per_keyframe =
            get_or<int>(m, "max_per_keyframe", c.max_measurements_per_keyframe);
        c.min_observations =
            get_or<int>(m, "min_observations", c.min_observations);
        c.max_pending_per_track =
            get_or<int>(m, "max_pending_per_track", c.max_pending_per_track);
    }
    return c;
}

struct LandmarkGraph::Impl {
    struct Keyframe {
        double stamp{0.0};
        /// Raw odometry pose.
        Eigen::Isometry3d odom_T_body{Eigen::Isometry3d::Identity()};
    };

    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    /// Latest estimate of every variable in isam, plus new_values.
    gtsam::Values estimate;

    std::vector<Keyframe> keyframes;
    std::map<int, int> observations;
    std::map<std::pair<int, std::size_t>, int> per_keyframe;

    static gtsam::ISAM2Params params() {
        gtsam::ISAM2Params p;
        p.relinearizeThreshold = 0.01;
        p.relinearizeSkip = 1;
        return p;
    }

    Impl() : isam(params()) {}

    std::size_t nearest_keyframe(double stamp) const {
        const auto it = std::lower_bound(
            keyframes.begin(), keyframes.end(), stamp,
            [](const Keyframe& k, double s) { return k.stamp < s; });
        if (it == keyframes.end()) {
            return keyframes.size() - 1;
        }
        const auto i = static_cast<std::size_t>(it - keyframes.begin());
        if (i > 0 && stamp - keyframes[i - 1].stamp < it->stamp - stamp) {
            return i - 1;
        }
        return i;
    }
};

LandmarkGraph::LandmarkGraph(LandmarkGraphConfig config)
    : config_(std::move(config)), impl_(std::make_unique<Impl>()) {}

LandmarkGraph::~LandmarkGraph() = default;

void LandmarkGraph::clear() {
    impl_ = std::make_unique<Impl>();
}

void LandmarkGraph::add_odometry(double stamp,
                                 const Eigen::Isometry3d& odom_T_body) {
    auto& kfs = impl_->keyframes;
    const gtsam::Pose3 pose = to_pose3(odom_T_body);
    const gtsam::Vector3 rpy = pose.rotation().rpy();
    const auto abs_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(
        config_.roll_pitch_std_deg * kDegToRad,
        config_.roll_pitch_std_deg * kDegToRad, config_.depth_std_m));

    if (kfs.empty()) {
        // The first keyframe defines the graph frame: odom as it is now.
        const gtsam::Key key = pose_key(0);
        const auto prior = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3)
                .finished());
        impl_->new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            key, pose, prior);
        impl_->new_factors.emplace_shared<AttitudeDepthFactor>(
            key, rpy(0), rpy(1), pose.z(), abs_noise);
        impl_->new_values.insert(key, pose);
        impl_->estimate.insert(key, pose);
        kfs.push_back({stamp, odom_T_body});
        return;
    }

    const auto& last = kfs.back();
    if (stamp <= last.stamp) {
        return;
    }
    const Eigen::Isometry3d delta = last.odom_T_body.inverse() * odom_T_body;
    const double dist = delta.translation().norm();
    const double angle = Eigen::AngleAxisd(delta.rotation()).angle();
    const double dt = stamp - last.stamp;
    const bool moved = dist >= config_.keyframe_distance_m ||
                       angle >= config_.keyframe_angle_deg * kDegToRad;
    const bool timed = config_.keyframe_interval_sec > 0.0 &&
                       dt >= config_.keyframe_interval_sec;
    if (!moved && !timed) {
        return;
    }

    const std::size_t index = kfs.size();
    const gtsam::Key prev = pose_key(index - 1);
    const gtsam::Key key = pose_key(index);
    const gtsam::Pose3 rel = to_pose3(delta);

    const double min_rot = config_.odom_min_rot_std_deg * kDegToRad;
    const double yaw_std =
        std::max(min_rot, (config_.odom_yaw_std_deg_per_m * dist +
                           config_.odom_yaw_std_deg_per_sec * dt) *
                              kDegToRad);
    const double pos_std =
        std::max(config_.odom_min_pos_std_m, config_.odom_pos_std_per_m * dist);
    // Tangent order: rotation (roll, pitch, yaw), then translation, both in
    // the frame of the previous keyframe.
    const auto between = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << min_rot, min_rot, yaw_std, pos_std, pos_std,
         pos_std)
            .finished());
    impl_->new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        prev, key, rel, between);
    impl_->new_factors.emplace_shared<AttitudeDepthFactor>(key, rpy(0), rpy(1),
                                                           pose.z(), abs_noise);

    const gtsam::Pose3 guess = impl_->estimate.at<gtsam::Pose3>(prev) * rel;
    impl_->new_values.insert(key, guess);
    impl_->estimate.insert(key, guess);
    kfs.push_back({stamp, odom_T_body});
}

bool LandmarkGraph::add_measurement(int landmark_id,
                                    double stamp,
                                    const Eigen::Vector3d& position,
                                    const Eigen::Matrix3d& covariance) {
    if (impl_->keyframes.empty()) {
        return false;
    }
    const std::size_t index = impl_->nearest_keyframe(stamp);
    int& count = impl_->per_keyframe[{landmark_id, index}];
    if (count >= config_.max_measurements_per_keyframe) {
        return false;
    }
    ++count;

    const auto& kf = impl_->keyframes[index];
    const Eigen::Matrix3d R = kf.odom_T_body.rotation();
    const Eigen::Vector3d in_body = kf.odom_T_body.inverse() * position;
    const Eigen::Matrix3d cov_body = R.transpose() * covariance * R;

    const auto gaussian = gtsam::noiseModel::Gaussian::Covariance(cov_body);
    const auto model = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(config_.huber_k),
        gaussian);

    const gtsam::Key pk = pose_key(index);
    const gtsam::Key lk = landmark_key(landmark_id);
    impl_->new_factors.emplace_shared<PointInBodyFactor>(pk, lk, in_body,
                                                         model);
    if (!impl_->estimate.exists(lk)) {
        const gtsam::Point3 guess =
            impl_->estimate.at<gtsam::Pose3>(pk).transformFrom(
                gtsam::Point3(in_body));
        impl_->new_values.insert(lk, guess);
        impl_->estimate.insert(lk, guess);
    }
    ++impl_->observations[landmark_id];
    return true;
}

void LandmarkGraph::optimize() {
    if (impl_->new_factors.empty()) {
        return;
    }
    impl_->isam.update(impl_->new_factors, impl_->new_values);
    impl_->new_factors.resize(0);
    impl_->new_values.clear();
    impl_->estimate = impl_->isam.calculateEstimate();
}

Eigen::Isometry3d LandmarkGraph::correction() const {
    if (impl_->keyframes.empty()) {
        return Eigen::Isometry3d::Identity();
    }
    const std::size_t last = impl_->keyframes.size() - 1;
    const gtsam::Pose3 graph_T_kf =
        impl_->estimate.at<gtsam::Pose3>(pose_key(last));
    return impl_->keyframes[last].odom_T_body *
           to_isometry(graph_T_kf).inverse();
}

std::optional<Eigen::Isometry3d> LandmarkGraph::latest_keyframe_estimate()
    const {
    if (impl_->keyframes.empty()) {
        return std::nullopt;
    }
    return to_isometry(impl_->estimate.at<gtsam::Pose3>(
        pose_key(impl_->keyframes.size() - 1)));
}

std::optional<Eigen::Vector3d> LandmarkGraph::landmark_in_odom(
    int landmark_id) const {
    if (observations(landmark_id) < config_.min_observations) {
        return std::nullopt;
    }
    const gtsam::Key lk = landmark_key(landmark_id);
    if (!impl_->estimate.exists(lk)) {
        return std::nullopt;
    }
    const Eigen::Vector3d p = impl_->estimate.at<gtsam::Point3>(lk);
    return correction() * p;
}

int LandmarkGraph::observations(int landmark_id) const {
    const auto it = impl_->observations.find(landmark_id);
    return it == impl_->observations.end() ? 0 : it->second;
}

std::size_t LandmarkGraph::keyframe_count() const {
    return impl_->keyframes.size();
}

std::size_t LandmarkGraph::landmark_count() const {
    return impl_->observations.size();
}

}  // namespace vortex::mission
