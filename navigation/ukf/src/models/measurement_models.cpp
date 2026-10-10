#include "ukf/models/measurement_models.hpp"

DVLMeasurementModel::DVLMeasurementModel(const DVLConfig& config)
    : velocity_noise_(config.velocity_noise) {}

DVLMeasurementModel::Point DVLMeasurementModel::h(const State& state,
                                                  const ImuInput& input) const {
    // TODO: body-frame velocity
    return Point::Zero();
}

Eigen::Matrix3d DVLMeasurementModel::R(const State& state,
                                       const ImuInput& input) const {
    return velocity_noise_;
}

DVLMeasurementModel::Point DVLMeasurementModel::composition_plus(
    const Point& measurement,
    const Eigen::Vector3d& delta) const {
    return measurement + delta;
}

Eigen::Vector3d DVLMeasurementModel::composition_minus(
    const Point& measurement_a,
    const Point& measurement_b) const {
    return measurement_a - measurement_b;
}

MagnetometerMeasurementModel::MagnetometerMeasurementModel(
    const MagnetometerConfig& config)
    : field_noise_(config.field_noise),
      reference_field_(config.reference_field) {}

MagnetometerMeasurementModel::Point MagnetometerMeasurementModel::h(
    const State& state,
    const ImuInput& input) const {
    // TODO: reference field rotated into the body frame
    return Point::Zero();
}

Eigen::Matrix3d MagnetometerMeasurementModel::R(const State& state,
                                                const ImuInput& input) const {
    return field_noise_;
}

MagnetometerMeasurementModel::Point
MagnetometerMeasurementModel::composition_plus(
    const Point& measurement,
    const Eigen::Vector3d& delta) const {
    return measurement + delta;
}

Eigen::Vector3d MagnetometerMeasurementModel::composition_minus(
    const Point& measurement_a,
    const Point& measurement_b) const {
    return measurement_a - measurement_b;
}

DepthMeasurementModel::DepthMeasurementModel(const DepthConfig& config)
    : depth_noise_variance_(config.depth_noise_variance),
      lever_arm_(config.lever_arm) {}

DepthMeasurementModel::Point DepthMeasurementModel::h(
    const State& state,
    const ImuInput& input) const {
    // TODO: sensor depth, the lever arm makes it depend on attitude
    return Point::Zero();
}

Eigen::Matrix<double, 1, 1> DepthMeasurementModel::R(
    const State& state,
    const ImuInput& input) const {
    return Eigen::Matrix<double, 1, 1>::Constant(depth_noise_variance_);
}

DepthMeasurementModel::Point DepthMeasurementModel::composition_plus(
    const Point& measurement,
    const Eigen::Matrix<double, 1, 1>& delta) const {
    return measurement + delta;
}

Eigen::Matrix<double, 1, 1> DepthMeasurementModel::composition_minus(
    const Point& measurement_a,
    const Point& measurement_b) const {
    return measurement_a - measurement_b;
}
