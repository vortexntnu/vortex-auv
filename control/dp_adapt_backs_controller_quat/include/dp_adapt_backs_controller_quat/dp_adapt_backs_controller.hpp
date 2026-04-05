#ifndef DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_HPP_
#define DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_HPP_

#include <eigen3/Eigen/Dense>
#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller_quat/typedefs.hpp"

namespace vortex::control {

struct DPAdaptParams {
    Eigen::Vector12d adapt_param = Eigen::Vector12d::Zero();
    Eigen::Vector6d d_gain = Eigen::Vector6d::Zero();
    Eigen::Vector6d K1 = Eigen::Vector6d::Zero();
    Eigen::Vector6d K2 = Eigen::Vector6d::Zero();
    Eigen::Vector3d r_b_bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d inertia_matrix_body = Eigen::Vector3d::Zero();
    Eigen::Matrix6d mass_intertia_matrix = Eigen::Matrix6d::Zero();
    double mass{};
};

class DPAdaptBacksController {
   public:
    explicit DPAdaptBacksController(const DPAdaptParams& dp_adapt_params);

    // @brief Calculates the control input tau found in the backstepping proof.
    // Utilizes error state to avoid 7x6 non invertible J matrix. The
    // approximation of quaternion error -> euler angle error is used, and
    // therefore we explicitly assume small pertrubations
    //
    // @param pose: 7D vector containing the vehicle pose [x, y, z, qw, qx, qy,
    // qz]
    // @param pose_d: 7D vector containing the desired vehicle pose [x, y, z,
    // qw, qx, qy, qz]
    // @param twist: 6D vector containing the vehicle velocity [u, v, w, p, q,
    // r]
    // @return 6D vector containing the control input tau [X, Y, Z, K, M, N]
    Eigen::Vector6d calculate_tau(const vortex::utils::types::Pose& pose,
                                  const vortex::utils::types::Pose& pose_d,
                                  const vortex::utils::types::Twist& twist);

    // @brief Reset the adaptive parameters
    void reset_adap_param();

    // @brief Reset the disturbance estimate
    void reset_d_est();

    // @brief Set the time step
    // @param dt: Time step
    void set_time_step(const double dt);

   private:
    Eigen::Matrix6d K1_;
    Eigen::Matrix6d K2_;
    Eigen::Vector3d r_b_bg_;
    Eigen::Matrix12d adapt_gain_;
    Eigen::Matrix6d d_gain_;
    Eigen::Vector12d adapt_param_;
    Eigen::Vector6d d_est_;
    Eigen::Matrix3d inertia_matrix_body_;
    Eigen::Matrix6d mass_intertia_matrix_;
    double m_{};
    double dt_{};
};

}  // namespace vortex::control

#endif  // DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_HPP_
