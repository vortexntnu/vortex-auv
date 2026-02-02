#ifndef ESKF_HPP
#define ESKF_HPP

#include <eigen3/Eigen/Dense>
#include <utility>
#include "typedefs.hpp"

struct VoDebug {
    double pos_innov_norm = 0.0;   // [m]
    double ang_innov_norm = 0.0;   // [rad]
    double nis_pose = 0.0;         // [-]
    bool   have = false;
};

class ESKF {
   public:
    ESKF(const EskfParams& params);

    // @brief Update the nominal state and error state
    // @param imu_meas: IMU measurement
    // @param dt: Time step
    void imu_update(const ImuMeasurement& imu_meas, const double dt);

    // @brief Update the nominal state and error state
    // @param dvl_meas: DVL measurement
    void dvl_update(const DvlMeasurement& dvl_meas);

    // @brief Update the nominal state and error state
    // @param visual_meas: Visual measurement
    void visualEgomotion_update(const VisualMeasurement& visual_meas);

    inline StateQuat get_nominal_state() const { return current_nom_state_; }

    inline double get_nis() const { return nis_; }
    bool have_vo_anchor = false;
    Eigen::Quaterniond q_nav_vo_{1, 0, 0, 0};
    Eigen::Vector3d p_nav_vo_{0, 0, 0};

    double last_vo_stamp_sec_ = 0.0;
    double vo_reset_gap_sec_ = 20.0;

    int consecutive_vo_rejects_ = 0;
    bool disable_nis_gating_ = false;

    const VoDebug& debug_vo() const { return debug_vo_; }

        /**
     * @brief Enable/disable NIS gating
     * When disabled, all measurements are accepted (for debugging)
     */
    void set_nis_gating_enabled(bool enabled);

        /**
     * @brief Force anchor reset on next VO measurement
     */
    void force_anchor_reset();
    
    /**
     * @brief Get consecutive rejection count
     */
    int get_consecutive_vo_rejects() const;
    
    /**
     * @brief Check if VO anchor is valid
     */
    bool is_anchor_valid() const;
    
    /**
     * @brief Set dropout detection gap threshold
     */
    void set_vo_reset_gap(double gap_sec);


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

    // @brief Update the error state
    // @param dvl_meas: DVL measurement
    void measurement_update(const DvlMeasurement& dvl_meas);

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

    // @brief Calculate the measurement matrix jakobian
    // @return Measurement matrix
    Eigen::Matrix3x19d calculate_hx();

    // @brief Calculate the full measurement matrix
    // @return Measurement matrix
    Eigen::Matrix3x18d calculate_h_jacobian();

    // @brief Calculate the measurement
    // @return Measurement
    Eigen::Vector3d calculate_h();

    // Process noise covariance matrix
    Eigen::Matrix12d Q_{};

    // Normalized Innovation Squared
    double nis_{};

    // Member variable for the current error state
    StateEuler current_error_state_{};

    // Member variable for the current nominal state
    StateQuat current_nom_state_{};

    void vo_velocity_update_(const Eigen::Vector3d& v_meas_nav,
                            const Eigen::Matrix3d& Rv);

    bool have_vo_prev{false};
    double prev_vo_stamp_sec{0.0};
    Eigen::Vector3d prev_p_meas_nav_{Eigen::Vector3d::Zero()};
    // Debug
    VoDebug debug_vo_;

};

#endif  // ESKF_HPP
