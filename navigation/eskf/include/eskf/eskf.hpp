#ifndef ESKF_HPP
#define ESKF_HPP

#include <eigen3/Eigen/Dense>
#include <utility>
#include "eskf/typedefs.hpp"
#include "typedefs.hpp"

class ESKF {
   public:
    ESKF(const eskf_params& params);

    std::pair<state_quat, state_euler> imu_update(
        const state_quat& nom_state,
        const state_euler& error_state,
        const imu_measurement& imu_meas,
        const double dt);

    std::pair<state_quat, state_euler> dvl_update(
        const state_quat& nom_state,
        const state_euler& error_state,
        const dvl_measurement& dvl_meas);

   private:
    // @brief Predict the nominal state
    // @param nom_state: Nominal state
    // @param imu_meas: IMU measurement
    // @return Predicted nominal state
    state_quat nominal_state_discrete(const state_quat& nom_state,
                                      const imu_measurement& imu_meas,
                                      const double dt);

    // @brief Predict the error state
    // @param error_state: Error state
    // @param nom_state: Nominal state
    // @param imu_meas: IMU measurement
    // @return Predicted error state
    state_euler error_state_prediction(const state_euler& error_state,
                                       const state_quat& nom_state,
                                       const imu_measurement& imu_meas,
                                       const double dt);

    // @brief Update the error state
    // @param error_state: Error state
    // @param dvl_meas: DVL measurement
    // @return Updated error state
    state_euler measurement_update(const state_quat& nom_state,
                                   const state_euler& error_state,
                                   const dvl_measurement& dvl_meas);

    // @brief Inject the error state into the nominal state and reset the error
    // state
    // @param nom_state: Nominal state
    // @param error_state: Error state
    // @return Injected and reset state
    std::pair<state_quat, state_euler> injection_and_reset(
        const state_quat& nom_state,
        const state_euler& error_state);

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
    Eigen::Matrix4x3d calculate_Q_delta(const state_quat& nom_state);

    // @brief Calculate the measurement matrix jakobian
    // @param nom_state: Nominal state
    // @return Measurement matrix
    Eigen::Matrix3x19d calculate_Hx(const state_quat& nom_state);

    // @brief Calculate the full measurement matrix
    // @param nom_state: Nominal state
    // @return Measurement matrix
    Eigen::Matrix3x18d calculate_H(const state_quat& nom_state);

    // @brief Calculate the measurement
    // @param nom_state: Nominal state
    // @return Measurement
    Eigen::Vector3d calculate_h(const state_quat& nom_state);

    Eigen::Matrix12d Q_;
};

#endif  // ESKF_HPP
