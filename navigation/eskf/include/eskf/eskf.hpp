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

    void depth_update(const SensorDepth& depth_meas);

    inline NominalState get_nominal_state() const { return current_nom_state_; }

    inline ErrorState get_error_state() const { return current_error_state_; }

    inline double get_nis_dvl() const { return nis_dvl_; }
    inline double get_nis_depth() const { return nis_depth_; }

    inline Eigen::Vector3d get_gravity() const { return g_; }

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

    // @brief Update the error state using a generic sensor measurement model
    // @tparam SensorT Type of the sensor model (must satisfy
    // SensorModelConcept)
    // @param meas Sensor measurement instance
    // @return NIS for this measurement update
    template <SensorModelConcept SensorT>
    double measurement_update(const SensorT& meas);

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

    double nis_dvl_{};
    double nis_depth_{};

    // Member variable for the current error state
    ErrorState current_error_state_{};

    // Member variable for the current nominal state
    NominalState current_nom_state_{};

    // gravity
    Eigen::Vector3d g_{0.0, 0.0, 9.82841};

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
Eigen::Vector3d calculate_h(const NominalState& current_nom_state_);

// Jacobian of h(x) with respect to the error state --> H
Eigen::Matrix3x15d calculate_h_jacobian(const NominalState& current_nom_state_);

// Jacobian of h(x) with respect to the nominal state --> Hx
Eigen::Matrix3x16d calculate_hx(const NominalState& current_nom_state_);

double compute_nis(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& S);

#include "eskf.tpp"  // including template implementation

#endif  // ESKF__ESKF_HPP_
