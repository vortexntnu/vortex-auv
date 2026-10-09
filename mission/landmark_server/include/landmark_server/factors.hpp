#ifndef LANDMARK_SERVER__FACTORS_HPP_
#define LANDMARK_SERVER__FACTORS_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace vortex::landmark_server {

/**
 * @brief Absolute depth of a pose (pressure sensor). Constrains only z.
 * Error: pose.z() - z_measured.
 */
class DepthFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
   public:
    DepthFactor(gtsam::Key key, double z, const gtsam::SharedNoiseModel& model)
        : NoiseModelFactorN(model, key), z_(z) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        boost::optional<gtsam::Matrix&> H = boost::none) const override {
        gtsam::Matrix36 H_t;
        const gtsam::Point3 t = pose.translation(H ? &H_t : nullptr);
        if (H) {
            *H = H_t.row(2);
        }
        return gtsam::Vector1(t.z() - z_);
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::make_shared<DepthFactor>(*this);
    }

   private:
    double z_;
};

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__FACTORS_HPP_
