#ifndef UKF__MODELS__MEASUREMENT_MODELS_HPP_
#define UKF__MODELS__MEASUREMENT_MODELS_HPP_

#include "ukf/typedefs.hpp"

struct DVLConfig {
    Eigen::Matrix3d velocity_noise = Eigen::Matrix3d::Identity();
};

class DVLMeasurementModel {
   public:
    static constexpr int dimension = 3;
    using Point = Eigen::Vector3d;

    explicit DVLMeasurementModel(const DVLConfig& config);

    Point h(const State& state, const ImuInput& input) const;

    Eigen::Matrix3d R(const State& state, const ImuInput& input) const;

    Point composition_plus(const Point& measurement,
                           const Eigen::Vector3d& delta) const;

    Eigen::Vector3d composition_minus(const Point& measurement_a,
                                      const Point& measurement_b) const;

   private:
    Eigen::Matrix3d velocity_noise_;
};

struct MagnetometerConfig {
    Eigen::Matrix3d field_noise = Eigen::Matrix3d::Identity();
    // local earth field in the navigation frame
    Eigen::Vector3d reference_field = Eigen::Vector3d::UnitX();
};

class MagnetometerMeasurementModel {
   public:
    static constexpr int dimension = 3;
    using Point = Eigen::Vector3d;

    explicit MagnetometerMeasurementModel(const MagnetometerConfig& config);

    Point h(const State& state, const ImuInput& input) const;

    Eigen::Matrix3d R(const State& state, const ImuInput& input) const;

    Point composition_plus(const Point& measurement,
                           const Eigen::Vector3d& delta) const;

    Eigen::Vector3d composition_minus(const Point& measurement_a,
                                      const Point& measurement_b) const;

   private:
    Eigen::Matrix3d field_noise_;
    Eigen::Vector3d reference_field_;
};

struct DepthConfig {
    double depth_noise_variance = 1.0;
    Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();
};

class DepthMeasurementModel {
   public:
    static constexpr int dimension = 1;
    using Point = Eigen::Matrix<double, 1, 1>;

    explicit DepthMeasurementModel(const DepthConfig& config);

    Point h(const State& state, const ImuInput& input) const;

    Eigen::Matrix<double, 1, 1> R(const State& state,
                                  const ImuInput& input) const;

    Point composition_plus(const Point& measurement,
                           const Eigen::Matrix<double, 1, 1>& delta) const;

    Eigen::Matrix<double, 1, 1> composition_minus(
        const Point& measurement_a,
        const Point& measurement_b) const;

   private:
    double depth_noise_variance_;
    Eigen::Vector3d lever_arm_;
};

#endif  // UKF__MODELS__MEASUREMENT_MODELS_HPP_
