#include "landmark_slam/graph.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include "landmark_slam/factors.hpp"

namespace vortex::landmark_slam {

namespace {

using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using gtsam::noiseModel::Robust;
using gtsam::noiseModel::mEstimator::DCS;

// Fixed noise values. Pose3 tangent order: roll, pitch, yaw, x, y, z.
constexpr double kAnchorSigma = 1e-3;               // X(0) without a prior map
constexpr double kAttitudeSigma = 0.02;             // IMU roll/pitch [rad]
constexpr double kDepthSigma = 0.05;                // pressure depth [m]
constexpr double kOdomRollPitchSigma = 0.01;        // per keyframe step [rad]
constexpr double kOdomZSigma = 0.02;                // per keyframe step [m]
constexpr double kOdomMinTransSigma = 0.01;         // floor per step [m]
constexpr double kOdomMinYawSigma = 0.002;          // floor per step [rad]
constexpr double kLevelSigma = 0.02;                // landmark roll/pitch [rad]
constexpr double kFreeYawSigma = std::numbers::pi;  // yaw nobody measures
constexpr double kFreeTransSigma = 100.0;  // new landmark: no position prior
constexpr double kOrientRollPitchSigma = 0.5;  // detector roll/pitch [rad]
constexpr double kOrientYawSigma = 0.1;        // detector yaw [rad]
constexpr double kDcsPhi = 1.0;                // Agarwal et al. 2013

gtsam::SharedNoiseModel sigmas(double rr,
                               double rp,
                               double ry,
                               double tx,
                               double ty,
                               double tz) {
    return Diagonal::Sigmas(
        (gtsam::Vector6() << rr, rp, ry, tx, ty, tz).finished());
}

gtsam::SharedNoiseModel robust(const gtsam::SharedNoiseModel& base) {
    return Robust::Create(DCS::Create(kDcsPhi), base);
}

double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

}  // namespace

gtsam::Vector3 bearing_range_sigmas(const Params& p, double range) {
    return {p.bearing_sigma, p.bearing_sigma, p.range_sigma(range)};
}

void LandmarkGraph::reset(const Config& cfg,
                          const gtsam::Pose3& T_odom_base,
                          double t) {
    cfg_ = cfg;
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(isam_params);
    new_factors_.resize(0);
    new_values_.clear();
    estimate_.clear();
    keyframes_.clear();
    landmarks_.clear();
    cache_.clear();

    // X(0): the start pose from the prior map, else odom (map = odom).
    gtsam::Pose3 X0 = T_odom_base;
    gtsam::SharedNoiseModel X0_noise = Isotropic::Sigma(6, kAnchorSigma);
    z_offset_ = 0.0;
    if (cfg.initial_pose) {
        const InitialPose& ip = *cfg.initial_pose;
        const gtsam::Vector3 rpy = T_odom_base.rotation().rpy();
        const double z = ip.z.value_or(T_odom_base.z());
        X0 = gtsam::Pose3(gtsam::Rot3::Ypr(ip.yaw, rpy(1), rpy(0)),
                          gtsam::Point3(ip.x, ip.y, z));
        X0_noise = sigmas(kAttitudeSigma, kAttitudeSigma, ip.sigma_yaw,
                          ip.sigma_xy, ip.sigma_xy, kDepthSigma);
        z_offset_ = z - T_odom_base.z();
    }
    new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), X0,
                                                                  X0_noise);
    new_values_.insert(X(0), X0);
    keyframes_.push_back({T_odom_base, t});
    add_absolute_factors(0, T_odom_base);

    update();
}

void LandmarkGraph::add_absolute_factors(int kf,
                                         const gtsam::Pose3& T_odom_base) {
    new_factors_.emplace_shared<DepthFactor>(X(kf), T_odom_base.z() + z_offset_,
                                             Isotropic::Sigma(1, kDepthSigma));
    // Gravity (map z axis) seen in the base frame, from the odometry's
    // roll and pitch.
    const gtsam::Unit3 b_gravity(
        T_odom_base.rotation().unrotate(gtsam::Point3(0.0, 0.0, 1.0)));
    new_factors_.emplace_shared<gtsam::Pose3AttitudeFactor>(
        X(kf), gtsam::Unit3(0.0, 0.0, 1.0), Isotropic::Sigma(2, kAttitudeSigma),
        b_gravity);
}

int LandmarkGraph::add_keyframe(const gtsam::Pose3& T_odom_base, double t) {
    const int prev = last_keyframe();
    const int kf = prev + 1;
    const gtsam::Pose3 delta =
        keyframes_.back().T_odom_base.between(T_odom_base);
    const double dist = delta.translation().norm();
    const Params& p = cfg_.params;
    const double trans =
        std::max(kOdomMinTransSigma, p.odom_sigma_trans_per_m * dist);
    const double yaw =
        std::max(kOdomMinYawSigma, p.odom_sigma_yaw_per_m * dist);
    new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(prev), X(kf), delta,
        sigmas(kOdomRollPitchSigma, kOdomRollPitchSigma, yaw, trans, trans,
               kOdomZSigma));
    new_values_.insert(X(kf), keyframe_pose(prev) * delta);
    keyframes_.push_back({T_odom_base, t});
    add_absolute_factors(kf, T_odom_base);
    return kf;
}

void LandmarkGraph::add_landmark(int id,
                                 const ClassConfig& cls,
                                 const gtsam::Pose3& init,
                                 double t) {
    // Keeps the landmark level and the problem well posed; the position and
    // (if measured) the yaw come from the observations.
    new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        L(id), init,
        sigmas(kLevelSigma, kLevelSigma, kFreeYawSigma, kFreeTransSigma,
               kFreeTransSigma, kFreeTransSigma));
    new_values_.insert(L(id), init);
    LandmarkState s;
    s.id = id;
    s.cls = cls;
    s.pose = init;
    s.first_seen = t;
    s.last_seen = t;
    landmarks_[id] = s;
}

void LandmarkGraph::add_observation(int kf,
                                    int id,
                                    const Measurement& m,
                                    double t) {
    LandmarkState& s = landmarks_.at(id);
    const Params& p = cfg_.params;
    const double range = m.position.norm();
    const double sym = s.cls.symmetry_deg;

    if (m.rotation && s.cls.has_orientation && sym < 360.0) {
        // Snap the measured yaw to the symmetric hypothesis closest to the
        // current estimate.
        gtsam::Rot3 R_base_obj = *m.rotation;
        if (sym > 0.0) {
            const double step = sym * std::numbers::pi / 180.0;
            const double measured =
                (keyframe_pose(kf).rotation() * R_base_obj).yaw();
            const double n = std::round(
                wrap(landmark_pose(id).rotation().yaw() - measured) / step);
            R_base_obj = R_base_obj * gtsam::Rot3::Yaw(n * step);
        }
        const double trans =
            std::max(p.range_sigma(range), p.bearing_sigma * range);
        new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(kf), L(id), gtsam::Pose3(R_base_obj, m.position),
            robust(sigmas(kOrientRollPitchSigma, kOrientRollPitchSigma,
                          kOrientYawSigma, trans, trans, trans)));
        s.yaw_known = true;
    } else {
        new_factors_.emplace_shared<BearingRange>(
            X(kf), L(id), gtsam::Unit3(m.position), range,
            robust(Diagonal::Sigmas(bearing_range_sigmas(p, range))));
    }
    if (s.n_obs == 0) {
        s.first_seen = t;
    }
    s.n_obs++;
    s.last_seen = std::max(s.last_seen, t);
}

void LandmarkGraph::update() {
    isam_->update(new_factors_, new_values_);
    new_factors_.resize(0);
    new_values_.clear();
    estimate_ = isam_->calculateEstimate();

    cache_.clear();
    for (auto& [id, s] : landmarks_) {
        s.pose = estimate_.at<gtsam::Pose3>(L(id));
        cache_.push_back(s);
    }
}

void LandmarkGraph::update_covariances() {
    if (landmarks_.empty()) {
        return;
    }
    const gtsam::Key xk = X(last_keyframe());
    gtsam::KeyVector keys{xk};
    for (const auto& [id, s] : landmarks_) {
        keys.push_back(L(id));
    }
    const gtsam::JointMarginal P = joint_cov(keys);
    const gtsam::Pose3 Xk = keyframe_pose(last_keyframe());
    const gtsam::Matrix3 R_mx = Xk.rotation().matrix();

    cache_.clear();
    for (auto& [id, s] : landmarks_) {
        s.cov = P(L(id), L(id));
        // Landmark position in the keyframe frame: d/dX and d/dL.
        gtsam::Matrix36 H_x;
        gtsam::Matrix36 H_lt;
        gtsam::Matrix3 H_p;
        Xk.transformTo(s.pose.translation(&H_lt), H_x, H_p);
        gtsam::Matrix H(3, 12);
        H << H_x, H_p * H_lt;
        gtsam::Matrix P_xl(12, 12);
        P_xl << P(xk, xk), P(xk, L(id)), P(L(id), xk), P(L(id), L(id));
        s.relative_cov = R_mx * (H * P_xl * H.transpose()) * R_mx.transpose();
        cache_.push_back(s);
    }
}

gtsam::Pose3 LandmarkGraph::keyframe_pose(int kf) const {
    if (new_values_.exists(X(kf))) {
        return new_values_.at<gtsam::Pose3>(X(kf));
    }
    return estimate_.at<gtsam::Pose3>(X(kf));
}

gtsam::Pose3 LandmarkGraph::landmark_pose(int id) const {
    if (new_values_.exists(L(id))) {
        return new_values_.at<gtsam::Pose3>(L(id));
    }
    return estimate_.at<gtsam::Pose3>(L(id));
}

gtsam::Pose3 LandmarkGraph::map_to_odom() const {
    const int kf = last_keyframe();
    return keyframe_pose(kf) * keyframes_.back().T_odom_base.inverse();
}

gtsam::JointMarginal LandmarkGraph::joint_cov(
    const gtsam::KeyVector& keys) const {
    const gtsam::Marginals marginals(isam_->getFactorsUnsafe(), estimate_);
    return marginals.jointMarginalCovariance(keys);
}

}  // namespace vortex::landmark_slam
