#ifndef ESKF_HPP
#define ESKF_HPP

#include <eigen3/Eigen/Dense>
#include <utility>
#include "eskf/typedefs.hpp"
#include "typedefs.hpp"

class ESKF {
   public:
    ESKF(const eskf_params& params);

    // @brief Update the nominal state and error state
    // @param imu_meas: IMU measurement
    // @param dt: Time step
    // @return Updated nominal state and error state
    std::pair<state_quat, state_euler> imu_update(
        const imu_measurement& imu_meas,
        const double dt);

    // @brief Update the nominal state and error state
    // @param dvl_meas: DVL measurement
    // @return Updated nominal state and error state
    std::pair<state_quat, state_euler> dvl_update(
        const dvl_measurement& dvl_meas);

    // NIS
    double NIS_;

    // NEEDS
    double NEES_;

    // ground truth
    state_quat ground_truth_;

   private:
    // @brief Predict the nominal state
    // @param imu_meas: IMU measurement
    // @param dt: Time step
    // @return Predicted nominal state
    void nominal_state_discrete(const imu_measurement& imu_meas,
                                const double dt);

    // @brief Predict the error state
    // @param imu_meas: IMU measurement
    // @param dt: Time step
    // @return Predicted error state
    void error_state_prediction(const imu_measurement& imu_meas,
                                const double dt);

    // @brief Calculate the NIS
    // @param innovation: Innovation vector
    // @param S: Innovation covariance matrix
    void NIS(const Eigen::Vector3d& innovation, const Eigen::Matrix3d& S);

    void NEEDS();

    // @brief Update the error state
    // @param dvl_meas: DVL measurement
    void measurement_update(const dvl_measurement& dvl_meas);

    // @brief Inject the error state into the nominal state and reset the error
    void injection_and_reset();

    // @brief Van Loan discretization
    // @param A_c: Continuous state transition matrix
    // @param G_c: Continuous input matrix
    // @return Discrete state transition matrix and discrete input matrix
    std::pair<Eigen::Matrix18d, Eigen::Matrix18d> van_loan_discretization(
        const Eigen::Matrix18d& A_c,
        const Eigen::Matrix18x12d& G_c,
        const double dt);

    // @brief Calculate the delta quaternion matrix
    // @param nom_state: Nominal state
    // @return Delta quaternion matrix
    Eigen::Matrix4x3d calculate_q_delta();

    // @brief Calculate the measurement matrix jakobian
    // @param nom_state: Nominal state
    // @return Measurement matrix
    Eigen::Matrix3x19d calculate_hx();

    // @brief Calculate the full measurement matrix
    // @param nom_state: Nominal state
    // @return Measurement matrix
    Eigen::Matrix3x18d calculate_h_jacobian();

    // @brief Calculate the measurement
    // @param nom_state: Nominal state
    // @return Measurement
    Eigen::Vector3d calculate_h();

    // Process noise covariance matrix
    Eigen::Matrix12d Q_;

    // Member variable for the current error state
    state_euler current_error_state_;

    // Member variable for the current nominal state
    state_quat current_nom_state_;
};

#endif  // ESKF_HPP
