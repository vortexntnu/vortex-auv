#include "gtsam_navigation/estimator.hpp"
#include "gtsam_navigation/dvl_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace gtsam_navigation {
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
static_assert(std::is_same_v<gtsam::DefaultPreintegrationType,
                             gtsam::TangentPreintegration>,
              "Build GTSAM with tangent preintegration ON and LieGroup OFF");

namespace {
gtsam::ISAM2Params isam_parameters() {
    gtsam::ISAM2Params params;
    params.findUnusedFactorSlots = true;
    params.relinearizeSkip = 1;
    params.setRelinearizeThreshold(0.01);
    params.setFactorization("QR");
    return params;
}

bool positive_covariance(const gtsam::Matrix3& matrix) {
    return matrix.allFinite() && matrix.isApprox(matrix.transpose(), 1e-8) &&
           Eigen::LLT<gtsam::Matrix3>(matrix).info() == Eigen::Success;
}
}  // namespace

Estimator::Estimator(const Config& config)
    : config_(config), smoother_(config.lag, isam_parameters()) {
    for (double value :
         {config.gravity, config.accel_noise_density, config.gyro_noise_density,
          config.accel_bias_random_walk, config.gyro_bias_random_walk,
          config.integration_sigma, config.lag, config.keyframe_interval,
          config.max_imu_gap, config.initialization_duration,
          config.stationary_accel_std, config.stationary_gyro_std,
          config.stationary_gyro_norm, config.stationary_gravity_tolerance,
          config.dvl_gate_squared, config.minimum_dvl_interval}) {
        if (!std::isfinite(value) || value <= 0) {
            throw std::invalid_argument(
                "Estimator parameters must be finite and positive");
        }
    }
    if (!std::isfinite(config.reorder_delay) || config.reorder_delay < 0 ||
        config.lag < 2 * config.keyframe_interval ||
        config.max_buffer_samples < 10 ||
        !config.imu_p_dvl.matrix().allFinite()) {
        throw std::invalid_argument(
            "Invalid lag, buffer size, reorder delay or sensor pose");
    }
    params_ = gtsam::PreintegrationParams::MakeSharedD(config.gravity);
    params_->setBodyPSensor(gtsam::Pose3());
    params_->setAccelerometerCovariance(
        gtsam::I_3x3 * std::pow(config.accel_noise_density, 2));
    params_->setGyroscopeCovariance(gtsam::I_3x3 *
                                    std::pow(config.gyro_noise_density, 2));
    params_->setIntegrationCovariance(gtsam::I_3x3 *
                                      std::pow(config.integration_sigma, 2));
    pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(params_);
}

void Estimator::fail(const std::string& reason) {
    status_.fault = true;
    status_.detail = reason;
    imu_buffer_.clear();
    dvl_buffer_.clear();
    status_.buffered_samples = 0;
}

bool Estimator::add_imu(const ImuSample& sample) {
    if (status_.fault) {
        return false;
    }
    if (!std::isfinite(sample.time) || sample.time < 0 ||
        !sample.acceleration.allFinite() ||
        !sample.angular_velocity.allFinite() ||
        (previous_imu_ && sample.time <= previous_imu_->time) ||
        imu_buffer_.count(sample.time)) {
        ++status_.rejected_imu;
        return false;
    }
    if (imu_buffer_.size() >= config_.max_buffer_samples) {
        fail("IMU buffer overflow; restart required");
        return false;
    }
    imu_buffer_.emplace(sample.time, sample);
    newest_imu_time_ = std::max(newest_imu_time_, sample.time);
    try {
        process();
    } catch (const std::exception& error) {
        fail(std::string("estimation failed; restart required: ") +
             error.what());
    }
    status_.buffered_samples = imu_buffer_.size() + dvl_buffer_.size();
    return !status_.fault;
}

bool Estimator::add_dvl(const DvlSample& sample) {
    if (status_.fault) {
        return false;
    }
    if (!std::isfinite(sample.time) || sample.time < 0 ||
        !sample.velocity.allFinite() ||
        !positive_covariance(sample.covariance) ||
        (previous_imu_ && sample.time <= previous_imu_->time) ||
        dvl_buffer_.count(sample.time) ||
        dvl_buffer_.size() >= config_.max_buffer_samples) {
        ++status_.rejected_dvl;
        return false;
    }
    dvl_buffer_.emplace(sample.time, sample);
    status_.buffered_samples = imu_buffer_.size() + dvl_buffer_.size();
    return true;
}

void Estimator::initialize(const ImuSample& sample) {
    alignment_.push_back(sample);
    // Retain the sample straddling the left boundary, including irregular
    // rates.
    while (alignment_.size() > 2 && sample.time - alignment_[1].time >=
                                        config_.initialization_duration) {
        alignment_.pop_front();
    }
    if (alignment_.size() > config_.max_buffer_samples) {
        fail("alignment buffer overflow; restart required");
        return;
    }
    if (alignment_.size() < 3 || sample.time - alignment_.front().time <
                                     config_.initialization_duration) {
        return;
    }
    gtsam::Vector3 mean_acc = gtsam::Vector3::Zero();
    gtsam::Vector3 mean_gyro = gtsam::Vector3::Zero();
    for (const auto& item : alignment_) {
        mean_acc += item.acceleration;
        mean_gyro += item.angular_velocity;
    }
    const double count = static_cast<double>(alignment_.size());
    mean_acc /= count;
    mean_gyro /= count;
    gtsam::Vector3 var_acc = gtsam::Vector3::Zero();
    gtsam::Vector3 var_gyro = gtsam::Vector3::Zero();
    for (const auto& item : alignment_) {
        var_acc += (item.acceleration - mean_acc).array().square().matrix();
        var_gyro +=
            (item.angular_velocity - mean_gyro).array().square().matrix();
    }
    if ((var_acc / count).maxCoeff() >
            std::pow(config_.stationary_accel_std, 2) ||
        (var_gyro / count).maxCoeff() >
            std::pow(config_.stationary_gyro_std, 2) ||
        mean_gyro.norm() > config_.stationary_gyro_norm ||
        std::abs(mean_acc.norm() - config_.gravity) >
            config_.stationary_gravity_tolerance) {
        alignment_.clear();
        status_.detail = "motion detected; restarting stationary alignment";
        return;
    }
    const double roll = std::atan2(-mean_acc.y(), -mean_acc.z());
    const double pitch =
        std::atan2(mean_acc.x(), std::hypot(mean_acc.y(), mean_acc.z()));
    anchor_.pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, 0.0),
                                gtsam::Point3::Zero());
    anchor_.time = sample.time;
    anchor_.bias =
        gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), mean_gyro);
    gtsam::Vector6 pose_sigmas;
    pose_sigmas << 0.02, 0.02, 0.001, 0.001, 0.001, 0.001;
    gtsam::Vector6 bias_sigmas;
    bias_sigmas << 0.01, 0.01, 0.01, 1e-4, 1e-4, 1e-4;
    gtsam::NonlinearFactorGraph graph;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(0), anchor_.pose, gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas)));
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
        V(0), anchor_.velocity, gtsam::noiseModel::Isotropic::Sigma(3, 0.01)));
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        B(0), anchor_.bias, gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas)));
    gtsam::Values values;
    values.insert(X(0), anchor_.pose);
    values.insert(V(0), anchor_.velocity);
    values.insert(B(0), anchor_.bias);
    smoother_.update(
        graph, values,
        {{X(0), sample.time}, {V(0), sample.time}, {B(0), sample.time}});
    anchor_.covariance.block<6, 6>(0, 0) =
        pose_sigmas.array().square().matrix().asDiagonal();
    anchor_.covariance.block<3, 3>(6, 6) = gtsam::I_3x3 * 1e-4;
    anchor_.covariance.block<6, 6>(9, 9) =
        bias_sigmas.array().square().matrix().asDiagonal();
    pim_->resetIntegrationAndSetBias(anchor_.bias);
    integrated_time_ = sample.time;
    alignment_.clear();
    status_.initialized = true;
    status_.detail = "initialized; waiting for bottom-track DVL";
    status_.active_states = 1;
    status_.factor_slots = smoother_.getFactors().size();
}

void Estimator::process() {
    const double watermark = newest_imu_time_ - config_.reorder_delay;
    while (!imu_buffer_.empty() && imu_buffer_.begin()->first <= watermark) {
        const auto next = imu_buffer_.begin()->second;
        imu_buffer_.erase(imu_buffer_.begin());
        if (previous_imu_) {
            latest_sample_dt_ = next.time - previous_imu_->time;
            if (latest_sample_dt_ > config_.max_imu_gap) {
                if (status_.initialized) {
                    fail("IMU time gap exceeded limit; restart required");
                    return;
                }
                alignment_.clear();
            }
        }
        if (!status_.initialized) {
            initialize(next);
        } else if (previous_imu_) {
            // Zero-order hold: sample at t is valid on [t, next_sample_time).
            while (integrated_time_ < next.time - 1e-9) {
                while (!dvl_buffer_.empty() &&
                       dvl_buffer_.begin()->first <= integrated_time_ + 1e-9) {
                    ++status_.rejected_dvl;
                    dvl_buffer_.erase(dvl_buffer_.begin());
                }
                double boundary = std::min(
                    next.time, anchor_.time + config_.keyframe_interval);
                if (!dvl_buffer_.empty()) {
                    boundary = std::min(boundary, dvl_buffer_.begin()->first);
                }
                integrate_to(boundary, *previous_imu_);
                if (!dvl_buffer_.empty() &&
                    std::abs(dvl_buffer_.begin()->first - boundary) < 1e-9) {
                    const auto dvl = dvl_buffer_.begin()->second;
                    dvl_buffer_.erase(dvl_buffer_.begin());
                    if (boundary - last_dvl_key_time_ >=
                        config_.minimum_dvl_interval) {
                        keyframe(dvl, *previous_imu_);
                        last_dvl_key_time_ = boundary;
                    } else {
                        ++status_.rejected_dvl;
                        if (boundary - anchor_.time >=
                            config_.keyframe_interval - 1e-9) {
                            keyframe(std::nullopt, *previous_imu_);
                        }
                    }
                } else if (boundary - anchor_.time >=
                           config_.keyframe_interval - 1e-9) {
                    keyframe(std::nullopt, *previous_imu_);
                }
            }
        }
        previous_imu_ = next;
        // DVL has no aiding role until the initial state is available.
        while (!dvl_buffer_.empty() &&
               dvl_buffer_.begin()->first <= next.time) {
            ++status_.rejected_dvl;
            dvl_buffer_.erase(dvl_buffer_.begin());
        }
    }
}

void Estimator::integrate_to(double time, const ImuSample& sample) {
