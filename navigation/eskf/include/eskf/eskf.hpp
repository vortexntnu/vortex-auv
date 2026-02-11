#ifndef ESKF_HPP
#define ESKF_HPP

#include <eigen3/Eigen/Dense>
#include <utility>
#include "typedefs.hpp"
#include <deque>

/**
 * @brief Error-State Kalman Filter for AUV state estimation.
 *
 * Fuses IMU, DVL, and visual landmark measurements to estimate
 * position, velocity, and orientation. Landmark processing includes
 * anchor-based relative positioning with a sliding-window smoother.
 */
class ESKF {
   public:
    ESKF(const EskfParams& params);

    /**
     * @brief IMU prediction step (propagates nominal and error state)
     *
     * @param imu_meas Accelerometer and gyroscope readings
     * @param dt Time since last IMU sample [s]
     */
    void imu_update(const ImuMeasurement& imu_meas, const double dt);

    /**
     * @brief DVL velocity measurement update
     *
     * @param dvl_meas Body-frame velocity with covariance
     */
    void dvl_update(const DvlMeasurement& dvl_meas);

    /**
     * @brief Landmark-based pose + velocity update
     *
     * Transforms the marker observation into the navigation frame via the
     * VO anchor, optionally smooths with the sliding window, derives a
     * velocity measurement, and performs a 6-DoF EKF update.
     *
     * @param visual_meas Marker pose in base frame with covariance
     */
    void landmark_update(const VisualMeasurement& visual_meas);

    inline StateQuat get_nominal_state() const { return current_nom_state_; }
    inline double get_nis() const { return nis_; }

    /**
     * @brief Load all VO tuning knobs at once
     */
    void set_vo_config(const VoConfig& cfg);

    void set_vo_enabled(bool enabled);

    /**
     * @brief Enable/disable NIS gating (disable for debugging)
     */
    void set_nis_gating_enabled(bool enabled);

    /**
     * @brief Force anchor reset on next VO measurement
     */
    void force_anchor_reset();

    /**
     * @brief Re-anchor when the tracked marker ID changes
     *
     * @param new_marker_pos  New marker position in base frame
     * @param new_marker_quat New marker orientation in base frame
     */
    void handle_marker_switch(const Eigen::Vector3d& new_marker_pos,
                              const Eigen::Quaterniond& new_marker_quat);

    int get_consecutive_rejects() const;
    bool is_anchor_valid() const;

   private:
    void nominal_state_discrete(const ImuMeasurement& imu_meas, const double dt);
    void error_state_prediction(const ImuMeasurement& imu_meas, const double dt);
    void measurement_update(const DvlMeasurement& dvl_meas);
    void injection_and_reset();

    std::pair<Eigen::Matrix18d, Eigen::Matrix18d> van_loan_discretization(
        const Eigen::Matrix18d& A_c,
        const Eigen::Matrix18x12d& G_c,
        const double dt);

    Eigen::Matrix3x19d calculate_hx();
    Eigen::Matrix3x18d calculate_h_jacobian();
    Eigen::Vector3d calculate_h();
    void NIS(const Eigen::Vector3d& innovation, const Eigen::Matrix3d& S);
    
    Eigen::Matrix12d Q_{};   
    double nis_{};           
    StateEuler current_error_state_{};
    StateQuat current_nom_state_{};

    bool use_vo_;
    bool disable_gating_;

    VoConfig vo_cfg_;        

    // True once the first marker has been seen
    bool have_anchor_;        

    // Rotation from VO frame to nav frame 
    Eigen::Quaterniond q_nav_vo_; 

    // Translation from VO frame to nav frame 
    Eigen::Vector3d p_nav_vo_;   
      
    double last_stamp_;     

    //Consecutive NIS-rejected VO updates 
    int consecutive_rejects_;

    // VO velocity from finite differences
    void landmark_vel_update_(const Eigen::Vector3d& v_meas_nav,
                              const Eigen::Matrix3d& Rv);
    bool have_prev_;
    double prev_stamp_;
    Eigen::Vector3d prev_p_nav_;

    // Sliding-window SO(3) smoother
    struct Sample {
        double stamp;
        Eigen::Vector3d pos;
        Eigen::Quaterniond quat;
    };
    std::deque<Sample> sw_buf_;

    void sw_add(double stamp, const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
    void sw_prune(double now);
    bool sw_estimate(Eigen::Vector3d& p_out, Eigen::Quaterniond& q_out);

    Eigen::Vector3d log_quat(const Eigen::Quaterniond& q);
    Eigen::Quaterniond exp_rotvec(const Eigen::Vector3d& w);
    double angle_dist(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b);
};

#endif  // ESKF_HPP
