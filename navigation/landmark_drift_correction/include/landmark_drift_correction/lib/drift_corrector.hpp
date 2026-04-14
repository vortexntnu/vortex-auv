#ifndef LANDMARK_DRIFT_CORRECTION__LIB__DRIFT_CORRECTOR_HPP_
#define LANDMARK_DRIFT_CORRECTION__LIB__DRIFT_CORRECTOR_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vortex::navigation::drift_correction {

struct Keyframe {
    std::size_t k;
    double t;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
};

class DriftCorrector {
   public:
    DriftCorrector() = default;

    void addKeyframe(double t,
                     const Eigen::Vector3d& p,
                     const Eigen::Quaterniond& q);

    const std::vector<Keyframe>& keyframes() const { return keyframes_; }
    std::size_t keyframe_count() const { return keyframes_.size(); }

   private:
    std::vector<Keyframe> keyframes_;
    std::size_t next_k_{0};
};

}  // namespace vortex::navigation::drift_correction

#endif  // LANDMARK_DRIFT_CORRECTION__LIB__DRIFT_CORRECTOR_HPP_
