#pragma once

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace gtsam_navigation {
using Matrix15 = Eigen::Matrix<double, 15, 15>;

struct ImuSample {
    double time;
    gtsam::Vector3 acceleration, angular_velocity;
};
struct DvlSample {
    double time;
    gtsam::Vector3 velocity;
    gtsam::Matrix3 covariance;
};

struct Config {
    double gravity = 9.81;
    double accel_noise_density = 0.07 / 60.0;
    double gyro_noise_density = 0.15 * M_PI / (180.0 * 60.0);
    // Bias random walks are tunable modeling assumptions, not bias instability.
    double accel_bias_random_walk = 1e-5;
    double gyro_bias_random_walk = 1e-7;
    double integration_sigma = 1e-5;
    double lag = 5.0;
    double keyframe_interval = 0.1;
    double reorder_delay = 0.25;
    double max_imu_gap = 0.1;
    double initialization_duration = 2.0;
    double stationary_accel_std = 0.15;
    double stationary_gyro_std = 0.005;
    double stationary_gyro_norm = 0.05;
    double stationary_gravity_tolerance = 0.3;
    double dvl_gate_squared = 16.27;
    double minimum_dvl_interval = 0.005;
    std::size_t max_buffer_samples = 20000;
    gtsam::Pose3 imu_p_dvl;
};

/** Internal IMU-origin estimate, never a bias ROS interface.
 * Covariance order: local Pose3 [rotation, translation], world velocity,
 * sensor-frame accelerometer bias, sensor-frame gyro bias.
 */
struct Estimate {
    double time = 0;
    gtsam::Pose3 pose;
    gtsam::Vector3 velocity = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias bias;
    Matrix15 covariance = Matrix15::Zero();
    gtsam::Vector3 angular_velocity = gtsam::Vector3::Zero();
    double gyro_sample_dt = 0.01;
};

struct Status {
    bool initialized = false;
    bool fault = false;
    std::string detail = "waiting for stationary IMU";
    std::size_t rejected_imu = 0, rejected_dvl = 0;
    std::size_t active_states = 0, factor_slots = 0, buffered_samples = 0;
    double last_dvl_time = -1;
};

/** Single-threaded, timestamp-driven estimator; the ROS wrapper serializes
 * calls. */
class Estimator {
   public:
    explicit Estimator(const Config& config);
    bool add_imu(const ImuSample& sample);
    bool add_dvl(const DvlSample& sample);
    std::optional<Estimate> latest() const;
    const Status& status() const { return status_; }
    const Config& config() const { return config_; }

   private:
    void process();
    void initialize(const ImuSample& sample);
    void integrate_to(double time, const ImuSample& sample);
    void keyframe(const std::optional<DvlSample>& dvl, const ImuSample& imu);
    Estimate predict(const gtsam::PreintegratedImuMeasurements& pim,
                     const ImuSample& imu,
                     double sample_dt) const;
    void fail(const std::string& reason);
