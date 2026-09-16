#include "eskf/gtsam_navigation.hpp"
#include <gtsam/config.h>
#include "eskf/eskf.hpp"
#include "eskf/gtsam_factors.hpp"
#ifdef GTSAM_TANGENT_PREINTEGRATION
#error \
    "Build GTSAM with GTSAM_TANGENT_PREINTEGRATION=OFF for the Forster manifold backend"
#endif
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <cmath>
#include <limits>
#include <stdexcept>
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
namespace {
using Bias = gtsam::imuBias::ConstantBias;
// Convert [pose rotation, pose local translation, nav velocity, ba, bg]
// into the existing ESKF covariance convention [nav p, nav v, local theta, bg,
// ba].
Eigen::Matrix15d output_jacobian(const gtsam::Rot3& rotation) {
    Eigen::Matrix15d J = Eigen::Matrix15d::Zero();
    J.block<3, 3>(0, 3) = rotation.matrix();
    J.block<3, 3>(3, 6).setIdentity();
    J.block<3, 3>(6, 0).setIdentity();
    J.block<3, 3>(9, 12).setIdentity();
    J.block<3, 3>(12, 9).setIdentity();
    return J;
}
class JointPrior
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, Bias> {
   public:
    JointPrior(const gtsam::Pose3& pose,
               const gtsam::Vector3& velocity,
               const Bias& bias,
               const Eigen::Matrix15d& covariance)
        : NoiseModelFactor3(gtsam::noiseModel::Gaussian::Covariance(covariance),
                            X(0),
                            V(0),
                            B(0)),
          pose_(pose),
          velocity_(velocity),
          bias_(bias) {}
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        const gtsam::Vector3& velocity,
        const Bias& bias,
        boost::optional<gtsam::Matrix&> Hpose = boost::none,
        boost::optional<gtsam::Matrix&> Hvelocity = boost::none,
        boost::optional<gtsam::Matrix&> Hbias = boost::none) const override {
        auto local = [this](const gtsam::Pose3& value) -> gtsam::Vector6 {
            return pose_.localCoordinates(value);
        };
        if (Hpose) {
            *Hpose = gtsam::Matrix::Zero(15, 6);
            Hpose->topRows<6>() =
                gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
                    local, pose);
        }
        if (Hvelocity) {
            *Hvelocity = gtsam::Matrix::Zero(15, 3);
            Hvelocity->block<3, 3>(6, 0).setIdentity();
        }
        if (Hbias) {
            *Hbias = gtsam::Matrix::Zero(15, 6);
            Hbias->bottomRows<6>().setIdentity();
        }
        gtsam::Vector result(15);
        result << local(pose), velocity - velocity_,
            bias.vector() - bias_.vector();
        return result;
    }

   private:
    gtsam::Pose3 pose_;
    gtsam::Vector3 velocity_;
    Bias bias_;
};
}  // namespace
struct GtsamNavigation::Impl {
    EskfParams params;
    GtsamNavigationParams options;
    gtsam::IncrementalFixedLagSmoother smoother;
    std::unique_ptr<gtsam::PreintegratedImuMeasurements> pim;
    std::unique_ptr<ESKF> predictor;
    gtsam::NavState anchor;
    Bias bias;
    double time = 0;
    size_t index = 0, active_states = 1;
    double nis_dvl = std::numeric_limits<double>::quiet_NaN();
    double nis_depth = std::numeric_limits<double>::quiet_NaN();
    bool healthy = true;
    std::string error;

    Impl(const EskfParams& p,
         const GtsamNavigationParams& o,
         const NominalState& initial)
        : params(p),
          options(o),
          smoother(o.lag_seconds),
          predictor(std::make_unique<ESKF>(p, initial)),
          anchor(
              gtsam::Pose3(gtsam::Rot3(initial.quat.normalized()), initial.pos),
              initial.vel),
          bias(initial.accel_bias, initial.gyro_bias) {
        if (!std::isfinite(o.lag_seconds) ||
            !std::isfinite(o.keyframe_interval) || o.keyframe_interval <= 0 ||
            o.lag_seconds <= o.keyframe_interval + p.max_imu_dt ||
            !std::isfinite(o.integration_covariance) ||
            o.integration_covariance <= 0 ||
            p.P.llt().info() != Eigen::Success ||
            p.Q.llt().info() != Eigen::Success)
            throw std::invalid_argument(
                "GTSAM requires positive-definite priors/noise, positive "
                "integration noise and lag > keyframe interval + max IMU dt");
        Eigen::Matrix12d blocks = Eigen::Matrix12d::Zero();
        for (int k = 0; k < 12; k += 3)
            blocks.block<3, 3>(k, k) = p.Q.block<3, 3>(k, k);
        if (!blocks.isApprox(p.Q, 1e-12))
            throw std::invalid_argument(
                "GTSAM IMU noise must be independent between its four 3D "
                "blocks");
        auto preintegration =
            boost::make_shared<gtsam::PreintegrationParams>(p.g_);
        preintegration->accelerometerCovariance =
            p.Q.block<3, 3>(NoiseIndex::acceleration, NoiseIndex::acceleration);
        preintegration->gyroscopeCovariance =
            p.Q.block<3, 3>(NoiseIndex::gyro, NoiseIndex::gyro);
        preintegration->integrationCovariance =
            Eigen::Matrix3d::Identity() * o.integration_covariance;
        pim = std::make_unique<gtsam::PreintegratedImuMeasurements>(
            preintegration, bias);
        const auto J = output_jacobian(anchor.attitude());
        gtsam::NonlinearFactorGraph factors;
        factors.emplace_shared<JointPrior>(anchor.pose(), anchor.v(), bias,
                                           J.transpose() * p.P * J);
        gtsam::Values values;
        values.insert(X(0), anchor.pose());
        values.insert(V(0), anchor.v());
        values.insert(B(0), bias);
        smoother.update(factors, values, {{X(0), 0}, {V(0), 0}, {B(0), 0}});
    }
    // Create a state at the current propagated epoch; never submit a
    // zero-duration IMU factor.
    void append_keyframe(gtsam::NonlinearFactorGraph& factors,
                         gtsam::Values& values,
                         gtsam::FixedLagSmoother::KeyTimestampMap& stamps) {
        const double dt = pim->deltaTij();
        if (dt <= 0)
            return;
        const size_t previous = index++;
        factors.emplace_shared<gtsam::ImuFactor>(
            X(previous), V(previous), X(index), V(index), B(previous), *pim);
        gtsam::Matrix6 covariance = gtsam::Matrix6::Zero();
        covariance.topLeftCorner<3, 3>() =
            params.Q.block<3, 3>(NoiseIndex::accel_bias,
                                 NoiseIndex::accel_bias) *
            dt;
        covariance.bottomRightCorner<3, 3>() =
            params.Q.block<3, 3>(NoiseIndex::gyro_bias, NoiseIndex::gyro_bias) *
            dt;
        factors.emplace_shared<gtsam::BetweenFactor<Bias>>(
            B(previous), B(index), Bias(),
            gtsam::noiseModel::Gaussian::Covariance(covariance));
        const auto predicted = pim->predict(anchor, bias);
        values.insert(X(index), predicted.pose());
        values.insert(V(index), predicted.v());
        values.insert(B(index), bias);
        stamps[X(index)] = time;
        stamps[V(index)] = time;
        stamps[B(index)] = time;
    }
    void solve(const gtsam::NonlinearFactorGraph& factors,
               const gtsam::Values& values,
               const gtsam::FixedLagSmoother::KeyTimestampMap& stamps) {
        smoother.update(factors, values, stamps);
        const auto estimate = smoother.calculateEstimate();
        anchor = gtsam::NavState(estimate.at<gtsam::Pose3>(X(index)),
                                 estimate.at<gtsam::Vector3>(V(index)));
        bias = estimate.at<Bias>(B(index));
        // Preserve full pose/velocity/bias cross-covariance when exporting.
        gtsam::Marginals marginals(smoother.getFactors(), estimate);
        const gtsam::KeyVector keys{X(index), V(index), B(index)};
        const auto joint = marginals.jointMarginalCovariance(keys);
        const int starts[] = {0, 6, 9}, sizes[] = {6, 3, 6};
        Eigen::Matrix15d covariance;
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
                covariance.block(starts[a], starts[b], sizes[a], sizes[b]) =
                    joint.at(keys[a], keys[b]);
        const auto J = output_jacobian(anchor.attitude());
        EskfParams propagated_params = params;
        propagated_params.P = J * covariance * J.transpose();
        propagated_params.P =
            (0.5 * (propagated_params.P + propagated_params.P.transpose()))
                .eval();
        NominalState state;
        state.quat = anchor.attitude().toQuaternion();
        state.pos = anchor.position();
        state.vel = anchor.v();
        state.accel_bias = bias.accelerometer();
        state.gyro_bias = bias.gyroscope();
        predictor = std::make_unique<ESKF>(propagated_params, state);
        pim->resetIntegrationAndSetBias(bias);
        active_states = estimate.size() / 3;
    }
    template <typename Sensor>
    bool gate(const Sensor& measurement, double threshold, double& nis) {
        const auto residual =
            measurement.innovation(predictor->get_nominal_state());
        const auto H = measurement.jacobian(predictor->get_nominal_state());
        const auto R = measurement.noise_covariance();
        nis = std::numeric_limits<double>::quiet_NaN();
        if (!residual.allFinite() || !H.allFinite() || !R.allFinite() ||
            !R.isApprox(R.transpose(), 1e-10) ||
            R.llt().info() != Eigen::Success)
            return false;
        const Eigen::Matrix<double, Sensor::dimension, Sensor::dimension> S =
            H * predictor->get_error_state().covariance * H.transpose() + R;
        const auto solver = S.llt();
        if (solver.info() != Eigen::Success)
            return false;
        nis = residual.dot(solver.solve(residual));
        return std::isfinite(nis) && nis >= 0 && nis <= threshold;
    }
};
GtsamNavigation::GtsamNavigation(const EskfParams& p,
                                 const GtsamNavigationParams& o,
                                 const NominalState& s)
    : impl_(std::make_unique<Impl>(p, o, s)) {}
GtsamNavigation::~GtsamNavigation() = default;
bool GtsamNavigation::imu_update(const ImuMeasurement& measurement, double dt) {
    if (!impl_->healthy || !impl_->predictor->imu_update(measurement, dt))
        return false;
    try {
        impl_->pim->integrateMeasurement(measurement.accel, measurement.gyro,
                                         dt);
        impl_->time += dt;
        if (impl_->pim->deltaTij() >= impl_->options.keyframe_interval) {
            gtsam::NonlinearFactorGraph factors;
            gtsam::Values values;
            gtsam::FixedLagSmoother::KeyTimestampMap stamps;
            impl_->append_keyframe(factors, values, stamps);
            impl_->solve(factors, values, stamps);
        }
        return true;
    } catch (const std::exception& error) {
        impl_->error = error.what();
        impl_->healthy = false;
        return false;
    }
}
bool GtsamNavigation::dvl_update(const SensorDVL& measurement) {
    if (!impl_->healthy ||
        !impl_->gate(measurement, impl_->params.dvl_nis_threshold,
                     impl_->nis_dvl))
        return false;
    try {
        gtsam::NonlinearFactorGraph factors;
        gtsam::Values values;
        gtsam::FixedLagSmoother::KeyTimestampMap stamps;
        impl_->append_keyframe(factors, values, stamps);
        factors.emplace_shared<eskf_gtsam::DvlFactor>(
            X(impl_->index), V(impl_->index), measurement);
        impl_->solve(factors, values, stamps);
        return true;
    } catch (const std::exception& error) {
        impl_->error = error.what();
        impl_->healthy = false;
        return false;
    }
}
bool GtsamNavigation::depth_update(const SensorDepth& measurement) {
    if (!impl_->healthy ||
        !impl_->gate(measurement, impl_->params.depth_nis_threshold,
                     impl_->nis_depth))
        return false;
    try {
        gtsam::NonlinearFactorGraph factors;
        gtsam::Values values;
        gtsam::FixedLagSmoother::KeyTimestampMap stamps;
        impl_->append_keyframe(factors, values, stamps);
        factors.emplace_shared<eskf_gtsam::DepthFactor>(X(impl_->index),
                                                        measurement);
        impl_->solve(factors, values, stamps);
        return true;
    } catch (const std::exception& error) {
        impl_->error = error.what();
        impl_->healthy = false;
        return false;
    }
}
NominalState GtsamNavigation::get_nominal_state() const {
    return impl_->predictor->get_nominal_state();
}
ErrorState GtsamNavigation::get_error_state() const {
    return impl_->predictor->get_error_state();
}
double GtsamNavigation::get_nis_dvl() const {
    return impl_->nis_dvl;
}
double GtsamNavigation::get_nis_depth() const {
    return impl_->nis_depth;
}
bool GtsamNavigation::healthy() const {
    return impl_->healthy;
}
size_t GtsamNavigation::keyframe_count() const {
    return impl_->index + 1;
}
size_t GtsamNavigation::active_state_count() const {
    return impl_->active_states;
}

std::string GtsamNavigation::error_message() const {
    return impl_->error;
}
std::optional<NominalState> GtsamNavigation::keyframe_state(
    size_t index) const {
    if (!impl_->smoother.getLinearizationPoint().exists(X(index)))
        return std::nullopt;
    const auto pose = impl_->smoother.calculateEstimate<gtsam::Pose3>(X(index));
    const auto bias = impl_->smoother.calculateEstimate<Bias>(B(index));
    NominalState state;
    state.pos = pose.translation();
    state.quat = pose.rotation().toQuaternion();
    state.vel = impl_->smoother.calculateEstimate<gtsam::Vector3>(V(index));
    state.gyro_bias = bias.gyroscope();
    state.accel_bias = bias.accelerometer();
    return state;
}
