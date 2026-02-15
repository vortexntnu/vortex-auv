#ifndef LANDMARK_EGOMOTION__LIB__LANDMARK_EGOMOTION_HPP_
#define LANDMARK_EGOMOTION__LIB__LANDMARK_EGOMOTION_HPP_

#include <deque>
#include <eskf/lib/eskf.hpp>
#include "vo_typedefs.hpp"

class LandmarkESKF : public ESKF {
   public:
    explicit LandmarkESKF(const EskfParams& params);
    ~LandmarkESKF() override = default;

    void landmark_update(const LandmarkMeasurement& Landmark_meas);
    void set_vo_config(const VoConfig& cfg);
    void set_vo_enabled(bool enabled);
    void set_nis_gating_enabled(bool enabled);
    void force_anchor_reset();
    void handle_marker_switch(const Eigen::Vector3d& pos,
                              const Eigen::Quaterniond& quat);
    int get_consecutive_rejects() const;
    bool is_anchor_valid() const;

   private:
    void landmark_vel_update_(const Eigen::Vector3d& v_meas_nav,
                              const Eigen::Matrix3d& Rv);

    static Eigen::Vector3d compute_quaternion_error(
        const Eigen::Quaterniond& q_meas,
        const Eigen::Quaterniond& q_est);
    static Eigen::Vector3d log_quat(const Eigen::Quaterniond& q);
    static Eigen::Quaterniond exp_rotvec(const Eigen::Vector3d& w);
    static double angle_dist(const Eigen::Quaterniond& a,
                             const Eigen::Quaterniond& b);

    static double compute_nis(const Eigen::VectorXd& innovation,
                              const Eigen::MatrixXd& S);

    // Sliding-window SO(3) smoother
    struct Sample {
        double stamp;
        Eigen::Vector3d pos;
        Eigen::Quaterniond quat;
    };
    std::deque<Sample> sw_buf_;

    void sw_add(double stamp,
                const Eigen::Vector3d& p,
                const Eigen::Quaterniond& q);
    void sw_prune(double now);
    bool sw_estimate(Eigen::Vector3d& p_out, Eigen::Quaterniond& q_out);

    bool use_vo_;
    bool disable_gating_;

    VoConfig vo_cfg_;

    bool have_anchor_;
    Eigen::Quaterniond q_nav_vo_;
    Eigen::Vector3d p_nav_vo_;

    double last_stamp_;
    int consecutive_rejects_;

    bool have_prev_;
    double prev_stamp_;
    Eigen::Vector3d prev_p_nav_;
};

#endif  // LANDMARK_EGOMOTION__LIB__LANDMARK_EGOMOTION_HPP_
