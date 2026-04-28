#ifndef LANDMARK_DRIFT_CORRECTION__LIB__DRIFT_CORRECTOR_HPP_
#define LANDMARK_DRIFT_CORRECTION__LIB__DRIFT_CORRECTOR_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "typedefs.hpp"

namespace vortex::navigation {

class DriftCorrector {
   public:
    /**
     * Constructs the DriftCorrector with the given noise parameters.
     * @param prior_noise_sigmas 6D vector of sigmas for the prior factor
     * (ordered [rx, ry, rz, tx, ty, tz], rad / m).
     * @param relative_odom_noise_sigmas 6D vector of sigmas for the odometry
     * factors (ordered [rx, ry, rz, tx, ty, tz], rad / m).
     */
    DriftCorrector(const Vector6d& prior_noise_sigmas,
                   const Vector6d& relative_odom_noise_sigmas);

    void addKeyframe(double t,
                     const Eigen::Vector3d& p,
                     const Eigen::Quaterniond& q);

    const std::vector<Keyframe>& keyframes() const { return keyframes_; }
    std::size_t keyframe_count() const { return keyframes_.size(); }

    // Returns the current ISAM2 optimized estimate for all keyframe poses.
    gtsam::Values currentEstimate() const;

   private:
    std::vector<Keyframe> keyframes_;
    std::size_t next_k_{1};  // X(0) is reserved for the prior anchor

    gtsam::ISAM2 isam2_;
    gtsam::NonlinearFactorGraph pending_factors_;
    gtsam::Values pending_values_;

    gtsam::noiseModel::Diagonal::shared_ptr relative_odom_noise_;
    gtsam::Pose3
        last_pose_;  // tracks X(k-1) for relative transform computation
};

}  // namespace vortex::navigation

#endif  // LANDMARK_DRIFT_CORRECTION__LIB__DRIFT_CORRECTOR_HPP_
