/**
 * @file measurement_models.hpp
 * @brief Contains the DVL, magnetometer and depth measurement models of the
 * UKF. All of them satisfy the SensorModel concept.
 */

#ifndef UKF__MODELS__MEASUREMENT_MODELS_HPP_
#define UKF__MODELS__MEASUREMENT_MODELS_HPP_

#include "ukf/typedefs.hpp"

/**
 * @brief Configuration of the DVLMeasurementModel.
 */
struct DVLConfig {
    Eigen::Matrix3d velocity_noise = Eigen::Matrix3d::Identity();
};

/**
 * @brief DVL, measuring the body-frame velocity.
 */
class DVLMeasurementModel {
   public:
    static constexpr int dimension = 3;
    using Point = Eigen::Vector3d;

    /**
     * @brief Constructor for the DVLMeasurementModel class.
     * @param config The noise configuration.
     */
    explicit DVLMeasurementModel(const DVLConfig& config);

    /**
     * @brief The measurement function, predicting the body-frame velocity.
     * @param state The state.
     * @param input The IMU measurement.
     * @return The predicted measurement.
     */
    Point h(const State& state, const ImuInput& input) const;

    /**
     * @brief The measurement noise covariance.
     * @param state The state.
     * @param input The IMU measurement.
     * @return The measurement noise covariance.
     */
    Eigen::Matrix3d R(const State& state, const ImuInput& input) const;

    /**
     * @brief Composes a measurement with a perturbation, plain addition.
     * @param measurement The measurement.
     * @param delta The perturbation of the measurement.
     * @return The composed measurement.
     */
    Point composition_plus(const Point& measurement,
                           const Eigen::Vector3d& delta) const;

    /**
     * @brief The difference between two measurements, plain subtraction.
     * @param measurement_a The measurement we subtract from.
     * @param measurement_b The measurement we subtract.
     * @return The delta between the two measurements.
     */
    Eigen::Vector3d composition_minus(const Point& measurement_a,
                                      const Point& measurement_b) const;

   private:
    Eigen::Matrix3d velocity_noise_;
};

/**
 * @brief Configuration of the MagnetometerMeasurementModel.
 */
struct MagnetometerConfig {
    Eigen::Matrix3d field_noise = Eigen::Matrix3d::Identity();
    Eigen::Vector3d reference_field = Eigen::Vector3d::UnitX();
};

/**
 * @brief Magnetometer, measuring the earth field in the body frame.
 */
class MagnetometerMeasurementModel {
   public:
    static constexpr int dimension = 3;
    using Point = Eigen::Vector3d;

    /**
     * @brief Constructor for the MagnetometerMeasurementModel class.
     * @param config The noise configuration.
     */
    explicit MagnetometerMeasurementModel(const MagnetometerConfig& config);

    /**
     * @brief The measurement function, predicting the reference field rotated
     * into the body frame.
     * @param state The state.
     * @param input The IMU measurement.
     * @return The predicted measurement.
     */
    Point h(const State& state, const ImuInput& input) const;

    /**
     * @brief The measurement noise covariance.
     * @param state The state.
     * @param input The IMU measurement.
     * @return The measurement noise covariance.
     */
    Eigen::Matrix3d R(const State& state, const ImuInput& input) const;

    /**
     * @brief Composes a measurement with a perturbation, plain addition.
     * @param measurement The measurement.
     * @param delta The perturbation of the measurement.
     * @return The composed measurement.
     */
    Point composition_plus(const Point& measurement,
                           const Eigen::Vector3d& delta) const;

    /**
     * @brief The difference between two measurements, plain subtraction.
     * @param measurement_a The measurement we subtract from.
     * @param measurement_b The measurement we subtract.
     * @return The delta between the two measurements.
     */
    Eigen::Vector3d composition_minus(const Point& measurement_a,
                                      const Point& measurement_b) const;

   private:
    Eigen::Matrix3d field_noise_;
    Eigen::Vector3d reference_field_;
};

/**
 * @brief Configuration of the DepthMeasurementModel.
 */
struct DepthConfig {
    double depth_noise_variance = 1.0;
    Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();
};

/**
 * @brief Depth sensor, measuring depth along the navigation z axis.
 */
class DepthMeasurementModel {
   public:
    static constexpr int dimension = 1;
    using Point = Eigen::Matrix<double, 1, 1>;

    /**
     * @brief Constructor for the DepthMeasurementModel class.
     * @param config The noise configuration.
     */
    explicit DepthMeasurementModel(const DepthConfig& config);

    /**
     * @brief The measurement function, predicting the depth of the sensor,
     * which depends on attitude through the lever arm.
     * @param state The state.
     * @param input The IMU measurement.
     * @return The predicted measurement.
     */
    Point h(const State& state, const ImuInput& input) const;

    /**
     * @brief The measurement noise covariance.
     * @param state The state.
     * @param input The IMU measurement.
     * @return The measurement noise covariance.
     */
    Eigen::Matrix<double, 1, 1> R(const State& state,
                                  const ImuInput& input) const;

    /**
     * @brief Composes a measurement with a perturbation, plain addition.
     * @param measurement The measurement.
     * @param delta The perturbation of the measurement.
     * @return The composed measurement.
     */
    Point composition_plus(const Point& measurement,
                           const Eigen::Matrix<double, 1, 1>& delta) const;

    /**
     * @brief The difference between two measurements, plain subtraction.
     * @param measurement_a The measurement we subtract from.
     * @param measurement_b The measurement we subtract.
     * @return The delta between the two measurements.
     */
    Eigen::Matrix<double, 1, 1> composition_minus(
        const Point& measurement_a,
        const Point& measurement_b) const;

   private:
    double depth_noise_variance_;
    Eigen::Vector3d lever_arm_;
};

#endif  // UKF__MODELS__MEASUREMENT_MODELS_HPP_
