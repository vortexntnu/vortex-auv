#include "landmark_server/graph.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include "landmark_server/factors.hpp"

namespace vortex::landmark_server {

namespace {

using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using gtsam::noiseModel::Robust;
using gtsam::noiseModel::mEstimator::DCS;

// Structural values, not tuning: the start defines the map frame, and a new
// landmark has no position prior and stays level. Pose3 tangent order:
// roll, pitch, yaw, x, y, z.
constexpr double kAnchorSigma = 1e-3;
constexpr double kLevelSigma = 0.02;
constexpr double kFreeYawSigma = std::numbers::pi;
constexpr double kFreeTransSigma = 100.0;

gtsam::SharedNoiseModel sigmas(double rr,
                               double rp,
                               double ry,
                               double tx,
                               double ty,
                               double tz) {
    return Diagonal::Sigmas(
        (gtsam::Vector6() << rr, rp, ry, tx, ty, tz).finished());
}

double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

}  // namespace

gtsam::Vector3 bearing_range_sigmas(const Params& p, double range) {
    return {p.bearing_sigma, p.bearing_sigma, p.range_sigma(range)};
}

gtsam::SharedNoiseModel LandmarkGraph::robust(
    const gtsam::SharedNoiseModel& base) const {
    return Robust::Create(DCS::Create(params_.dcs_phi), base);
}

void LandmarkGraph::reset(const Params& params,
                          const gtsam::Pose3& T_odom_base,
                          double t) {
    params_ = params;
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(isam_params);
    new_factors_.resize(0);
    new_values_.clear();
    estimate_.clear();
    marginals_.reset();
    keyframes_.clear();
    landmarks_.clear();
    retired_.clear();
    cache_.clear();

    // X(0): the start, levelled by the odometry's roll and pitch, at the
    // pressure depth.
    const gtsam::Vector3 rpy = T_odom_base.rotation().rpy();
    const gtsam::Pose3 X0(gtsam::Rot3::Ypr(0.0, rpy(1), rpy(0)),
                          gtsam::Point3(0.0, 0.0, T_odom_base.z()));
    new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(0), X0, Isotropic::Sigma(6, kAnchorSigma));
    new_values_.insert(X(0), X0);
    keyframes_.push_back({T_odom_base, t});
    add_absolute_factors(0, T_odom_base);
    update();
}

void LandmarkGraph::add_absolute_factors(int kf,
                                         const gtsam::Pose3& T_odom_base) {
    new_factors_.emplace_shared<DepthFactor>(
        X(kf), T_odom_base.z(), Isotropic::Sigma(1, params_.depth_sigma));
    // Gravity (map z axis) seen in the base frame, from the odometry's
    // roll and pitch.
    const gtsam::Unit3 b_gravity(
        T_odom_base.rotation().unrotate(gtsam::Point3(0.0, 0.0, 1.0)));
    new_factors_.emplace_shared<gtsam::Pose3AttitudeFactor>(
        X(kf), gtsam::Unit3(0.0, 0.0, 1.0),
        Isotropic::Sigma(2, params_.attitude_sigma), b_gravity);
}

int LandmarkGraph::add_keyframe(const gtsam::Pose3& T_odom_base, double t) {
    const int prev = last_keyframe();
    const int kf = prev + 1;
    const gtsam::Pose3 delta =
        keyframes_.back().T_odom_base.between(T_odom_base);
    const double dist = delta.translation().norm();
    const Params& p = params_;
    const double trans =
        std::max(p.odom_min_sigma_trans, p.odom_sigma_trans_per_m * dist);
    const double yaw =
        std::max(p.odom_min_sigma_yaw, p.odom_sigma_yaw_per_m * dist);
    new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(prev), X(kf), delta,
        sigmas(p.odom_sigma_roll_pitch, p.odom_sigma_roll_pitch, yaw, trans,
               trans, p.odom_sigma_z));
    new_values_.insert(X(kf), keyframe_pose(prev) * delta);
    keyframes_.push_back({T_odom_base, t});
    add_absolute_factors(kf, T_odom_base);
    return kf;
}

void LandmarkGraph::add_landmark(int id,
                                 const ClassConfig& cls,
                                 const gtsam::Pose3& init,
                                 double t) {
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
    const Params& p = params_;
    const double range = m.position.norm();
    const double sym = s.cls.symmetry_deg;
    const auto merged = std::min<std::size_t>(
        m.merged, static_cast<std::size_t>(p.max_merged_per_factor));
    const double scale = std::sqrt(static_cast<double>(merged));

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
            std::max(p.range_sigma(range), p.bearing_sigma * range) / scale;
        new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(kf), L(id), gtsam::Pose3(R_base_obj, m.position),
            robust(sigmas(p.orientation_roll_pitch_sigma,
                          p.orientation_roll_pitch_sigma,
                          p.orientation_yaw_sigma, trans, trans, trans)));
        s.yaw_known = true;
    } else {
        new_factors_.emplace_shared<BearingRange>(
            X(kf), L(id), gtsam::Unit3(m.position), range,
            robust(Diagonal::Sigmas(bearing_range_sigmas(p, range) / scale)));
    }
    if (s.n_obs == 0) {
        s.first_seen = t;
    }
    s.n_obs++;
    s.last_seen = std::max(s.last_seen, t);
}

void LandmarkGraph::retire(int id) {
    retired_.insert(id);
    cache_.erase(
        std::remove_if(cache_.begin(), cache_.end(),
                       [id](const LandmarkState& l) { return l.id == id; }),
        cache_.end());
}

std::vector<std::pair<int, int>> LandmarkGraph::retire_duplicates(
    double max_dist_m,
    double d2_gate) {
    std::vector<std::pair<int, int>> pairs;
    gtsam::KeyVector keys;
    for (const LandmarkState& a : cache_) {
        for (const LandmarkState& b : cache_) {
            if (a.id < b.id && a.cls.name == b.cls.name &&
                (a.pose.translation() - b.pose.translation()).norm() <
                    max_dist_m) {
                pairs.emplace_back(a.id, b.id);
                for (const int id : {a.id, b.id}) {
                    if (std::find(keys.begin(), keys.end(), L(id)) ==
                        keys.end()) {
                        keys.push_back(L(id));
                    }
                }
            }
        }
    }
    std::vector<std::pair<int, int>> out;
    if (pairs.empty()) {
        return out;
    }
    const gtsam::JointMarginal P = joint_cov(keys);
    for (const auto& [ia, ib] : pairs) {
        if (retired_.count(ia) > 0 || retired_.count(ib) > 0) {
            continue;
        }
        const LandmarkState& a = landmarks_.at(ia);
        const LandmarkState& b = landmarks_.at(ib);
        // Covariance of a - b in the map frame (the tangent translation is
        // in each landmark's own frame).
        const gtsam::Matrix3 Ra = a.pose.rotation().matrix();
        const gtsam::Matrix3 Rb = b.pose.rotation().matrix();
        const auto t = [&](int i, int j) {
            return gtsam::Matrix3(P(L(i), L(j)).block<3, 3>(3, 3));
        };
        const gtsam::Matrix3 S =
            Ra * t(ia, ia) * Ra.transpose() + Rb * t(ib, ib) * Rb.transpose() -
            Ra * t(ia, ib) * Rb.transpose() - Rb * t(ib, ia) * Ra.transpose();
        const gtsam::Vector3 d = a.pose.translation() - b.pose.translation();
        if (d.dot(S.ldlt().solve(d)) > d2_gate) {
            continue;
        }
        const bool a_keeps = a.n_obs >= b.n_obs;
        const int keep = a_keeps ? ia : ib;
        const int drop = a_keeps ? ib : ia;
        retire(drop);
        out.emplace_back(keep, drop);
    }
    return out;
}

void LandmarkGraph::update() {
    isam_->update(new_factors_, new_values_);
    new_factors_.resize(0);
    new_values_.clear();
    estimate_ = isam_->calculateEstimate();
    marginals_.reset();

    cache_.clear();
    for (auto& [id, s] : landmarks_) {
        s.pose = estimate_.at<gtsam::Pose3>(L(id));
        if (retired_.count(id) == 0) {
            cache_.push_back(s);
        }
    }
}

void LandmarkGraph::update_covariances() {
    if (cache_.empty()) {
        return;
    }
    const gtsam::Key xk = X(last_keyframe());
    gtsam::KeyVector keys{xk};
    for (const LandmarkState& s : cache_) {
        keys.push_back(L(s.id));
    }
    const gtsam::JointMarginal P = joint_cov(keys);
    const gtsam::Pose3 Xk = keyframe_pose(last_keyframe());
    const gtsam::Matrix3 R_mx = Xk.rotation().matrix();

    for (LandmarkState& s : cache_) {
        s.cov = P(L(s.id), L(s.id));
        // Landmark position in the keyframe frame: d/dX and d/dL.
        gtsam::Matrix36 H_x;
        gtsam::Matrix36 H_lt;
        gtsam::Matrix3 H_p;
        Xk.transformTo(s.pose.translation(&H_lt), H_x, H_p);
        gtsam::Matrix H(3, 12);
        H << H_x, H_p * H_lt;
        gtsam::Matrix P_xl(12, 12);
        P_xl << P(xk, xk), P(xk, L(s.id)), P(L(s.id), xk), P(L(s.id), L(s.id));
        s.relative_cov = R_mx * (H * P_xl * H.transpose()) * R_mx.transpose();
        landmarks_.at(s.id).cov = s.cov;
        landmarks_.at(s.id).relative_cov = s.relative_cov;
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
    return keyframe_pose(last_keyframe()) *
           keyframes_.back().T_odom_base.inverse();
}

gtsam::JointMarginal LandmarkGraph::joint_cov(
    const gtsam::KeyVector& keys) const {
    // One factorisation per estimate: every association and the published
    // covariances until the next update share it.
    if (!marginals_) {
        marginals_.emplace(isam_->getFactorsUnsafe(), estimate_);
    }
    return marginals_->jointMarginalCovariance(keys);
}

}  // namespace vortex::landmark_server
