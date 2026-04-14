#include "landmark_drift_correction/lib/drift_corrector.hpp"

namespace vortex::navigation::drift_correction {

void DriftCorrector::addKeyframe(double t,
                                 const Eigen::Vector3d& p,
                                 const Eigen::Quaterniond& q) {
    keyframes_.push_back(Keyframe{next_k_++, t, p, q.normalized()});
}

}  // namespace vortex::navigation::drift_correction
