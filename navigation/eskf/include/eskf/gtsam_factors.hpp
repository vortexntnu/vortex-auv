#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "eskf/lie.hpp"
#include "eskf/typedefs.hpp"

namespace eskf_gtsam {
class DvlFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
   public:
    DvlFactor(gtsam::Key pose, gtsam::Key velocity, const SensorDVL& sensor)
        : NoiseModelFactor2(
              gtsam::noiseModel::Gaussian::Covariance(sensor.measurement_noise),
              pose,
              velocity),
          measurement_(sensor.measurement) {}
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        const gtsam::Vector3& velocity,
        boost::optional<gtsam::Matrix&> Hpose = boost::none,
        boost::optional<gtsam::Matrix&> Hvelocity =
            boost::none) const override {
        const gtsam::Vector3 predicted = pose.rotation().unrotate(velocity);
        if (Hpose) {
            *Hpose = gtsam::Matrix::Zero(3, 6);
            Hpose->leftCols<3>() = eskf_math::skew(predicted);
        }
        if (Hvelocity)
            *Hvelocity = pose.rotation().transpose();
        return predicted - measurement_;
    }

   private:
    gtsam::Vector3 measurement_;
};
class DepthFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
   public:
    DepthFactor(gtsam::Key pose, const SensorDepth& sensor)
        : NoiseModelFactor1(
              gtsam::noiseModel::Isotropic::Variance(1,
                                                     sensor.measurement_noise),
              pose),
          measurement_(sensor.measurement),
          lever_arm_(sensor.lever_arm) {}
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        boost::optional<gtsam::Matrix&> Hpose = boost::none) const override {
        if (Hpose) {
            *Hpose = gtsam::Matrix::Zero(1, 6);
            Hpose->leftCols<3>() =
                -(pose.rotation().matrix() * eskf_math::skew(lever_arm_))
                     .row(2);
            Hpose->rightCols<3>() = pose.rotation().matrix().row(2);
        }
        return gtsam::Vector1(pose.translation().z() +
                              pose.rotation().rotate(lever_arm_).z() -
                              measurement_);
    }

   private:
    double measurement_;
    gtsam::Vector3 lever_arm_;
};
}  // namespace eskf_gtsam
