#include "landmark_drift_correction/lib/drift_corrector.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "landmark_drift_correction/lib/gtsam_conversions.hpp"

using gtsam::symbol_shorthand::X;

namespace vortex::navigation {

DriftCorrector::DriftCorrector(const gtsam::Vector6& prior_noise_sigmas,
                               const gtsam::Vector6& relative_odom_noise_sigmas)
    : last_pose_(gtsam::Pose3::Identity()) {
    gtsam::ISAM2Params params;
    isam2_ = gtsam::ISAM2(params);

    relative_odom_noise_ =
        gtsam::noiseModel::Diagonal::Sigmas(relative_odom_noise_sigmas);

    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(prior_noise_sigmas);
    pending_factors_.addPrior(X(0), last_pose_, prior_noise);
    pending_values_.insert(X(0), last_pose_);
    isam2_.update(pending_factors_, pending_values_);
    pending_factors_ = gtsam::NonlinearFactorGraph{};
    pending_values_.clear();
}

void DriftCorrector::addKeyframe(double t,
                                 const Eigen::Vector3d& p,
                                 const Eigen::Quaterniond& q) {
    const std::size_t k = next_k_++;
    const gtsam::Pose3 pose = to_gtsam_pose(p, q.normalized());
    const gtsam::Pose3 relative = last_pose_.between(pose);

    pending_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        X(k - 1), X(k), relative, relative_odom_noise_));
    pending_values_.insert(X(k), pose);

    isam2_.update(pending_factors_, pending_values_);
    pending_factors_ = gtsam::NonlinearFactorGraph{};
    pending_values_.clear();

    last_pose_ = pose;
    keyframes_.push_back(Keyframe{k, t, p, q.normalized()});
}

gtsam::Values DriftCorrector::currentEstimate() const {
    return isam2_.calculateEstimate();
}

}  // namespace vortex::navigation
