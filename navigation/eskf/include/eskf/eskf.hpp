#ifndef ESKF__ESKF_HPP_
#define ESKF__ESKF_HPP_
#include <limits>
#include "eskf/typedefs.hpp"

class ESKF {
   public:
    explicit ESKF(const EskfParams& params, const NominalState& initial = {});
    // Rejected inputs leave both state and covariance unchanged.
    bool imu_update(const ImuMeasurement& measurement, double dt);
    bool dvl_update(const SensorDVL& measurement);
    bool depth_update(const SensorDepth& measurement);
    NominalState get_nominal_state() const { return current_nom_state_; }
    ErrorState get_error_state() const { return current_error_state_; }
    double get_nis_dvl() const { return nis_dvl_; }
    double get_nis_depth() const { return nis_depth_; }
    Eigen::Vector3d get_gravity() const { return params_.g_; }

   private:
    void nominal_state_discrete(const ImuMeasurement& measurement, double dt);
    void error_state_prediction(const ImuMeasurement& measurement, double dt);
    void injection_and_reset();
    template <SensorModelConcept Sensor>
    bool measurement_update(const Sensor& measurement,
                            double threshold,
                            double& nis);
    EskfParams params_;
    NominalState current_nom_state_;
    ErrorState current_error_state_;
    double nis_dvl_ = std::numeric_limits<double>::quiet_NaN();
    double nis_depth_ = std::numeric_limits<double>::quiet_NaN();
};
Eigen::Vector3d calculate_h(const NominalState& state);
Eigen::Matrix3x15d calculate_h_jacobian(const NominalState& state);
#include "eskf/eskf.tpp"
#endif  // ESKF__ESKF_HPP_
