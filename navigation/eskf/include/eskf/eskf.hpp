#ifndef ESKF__ESKF_HPP_
#define ESKF__ESKF_HPP_

#include <eigen3/Eigen/Dense>
#include <utility>
#include "eskf/typedefs.hpp"
#include "typedefs.hpp"

class ESKF {
   public:
    explicit ESKF(const EskfParams& params);

    // @brief Update the nominal state and error state
    // @param imu_meas: IMU measurement
    // @param dt: Time step
    void imu_update(const ImuMeasurement& imu_meas, const double dt);

    // @brief Update the nominal state and error state
    // @param dvl_meas: DVL measurement
    void dvl_update(const SensorDVL& dvl_meas);

    inline StateQuat get_nominal_state() const { return current_nom_state_; }

    inline StateEuler get_error_state() const { return current_error_state_; }

    inline double get_nis() const { return nis_; }

   private:
    // @brief Predict the nominal state
    // @param imu_meas: IMU measurement
    // @param dt: Time step
    // @return Predicted nominal state
    void nominal_state_discrete(const ImuMeasurement& imu_meas,
                                const double dt);

    // @brief Predict the error state
    // @param imu_meas: IMU measurement
    // @param dt: Time step
    // @return Predicted error state
    void error_state_prediction(const ImuMeasurement& imu_meas,
                                const double dt);

    // @brief Calculate the NIS
    // @param innovation: Innovation vector
    // @param S: Innovation covariance matrix
    void NIS(const Eigen::Vector3d& innovation, const Eigen::Matrix3d& S);

    // @brief Update the error state using a generic sensor measurement model
    // @tparam SensorT Type of the sensor model (must satisfy
    // SensorModelConcept)
    // @param meas Sensor measurement instance
    template <SensorModelConcept SensorT>
    void measurement_update(const SensorT& meas);

    // @brief Inject the error state into the nominal state and reset the error
    void injection_and_reset();

    // @brief Van Loan discretization
    // @param A_c: Continuous state transition matrix
    // @param G_c: Continuous input matrix
    // @return Discrete state transition matrix and discrete input matrix
    std::pair<Eigen::Matrix15d, Eigen::Matrix15d> van_loan_discretization(
        const Eigen::Matrix15d& A_c,
        const Eigen::Matrix15x12d& G_c,
        const double dt);

    // Process noise covariance matrix
    Eigen::Matrix12d Q_{};

    // Normalized Innovation Squared
    double nis_{};

    // Member variable for the current error state
    StateEuler current_error_state_{};

    // Member variable for the current nominal state
    StateQuat current_nom_state_{};

    // gravity
    Eigen::Vector3d g_{0.0, 0.0, 9.82};

    // accelometer noise parameters
    float accm_std_{0.0};
    float accm_bias_std_{0.0};
    float accm_bias_p_{1e-16};

    // gyroscope noise parameters
    float gyro_std_{0.0};
    float gyro_bias_std_{0.0};
    float gyro_bias_p_{1e-16};
};

// Measurement in world frame --> h(x)
Eigen::Vector3d calculate_h(const StateQuat& current_nom_state_);

// Jacobian of h(x) with respect to the error state --> H
Eigen::Matrix3x15d calculate_h_jacobian(const StateQuat& current_nom_state_);

// Jacobian of h(x) with respect to the nominal state --> Hx
Eigen::Matrix3x16d calculate_hx(const StateQuat& current_nom_state_);

double compute_nis(const Eigen::Vector3d& innovation, const Eigen::Matrix3d& S);

#include "eskf.tpp"  // including template implementation

#endif  // ESKF__ESKF_HPP_
