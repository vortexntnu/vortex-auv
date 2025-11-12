#ifndef DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_HPP_
#define DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_HPP_

#include <eigen3/Eigen/Dense>
#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller/typedefs.hpp"

namespace vortex::control {

struct DPAdaptParams {
    Eigen::Vector12d adapt_param = Eigen::Vector12d::Zero();
    Eigen::Vector6d d_gain = Eigen::Vector6d::Zero();
    Eigen::Vector6d K1 = Eigen::Vector6d::Zero();
    Eigen::Vector6d K2 = Eigen::Vector6d::Zero();
    Eigen::Vector3d r_b_bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d I_b = Eigen::Vector3d::Zero();
    Eigen::Matrix6d mass_matrix = Eigen::Matrix6d::Zero();
    double mass{};
};

class DPAdaptBacksController {
   public:
    explicit DPAdaptBacksController(const DPAdaptParams& dp_adapt_params);

    // @brief Calculate thecontrol input tau
    // @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch,
    // yaw]
    // @param eta_d: 6D vector containing the desired vehicle pose [x, y, z,
    // roll, pitch, yaw]
    // @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
    // @return 6D vector containing the control input tau [X, Y, Z, K, M, N]
    Eigen::Vector6d calculate_tau(const vortex::utils::types::Eta& eta,
                                  const vortex::utils::types::Eta& eta_d,
                                  const vortex::utils::types::Nu& nu);

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
    Eigen::Matrix3d I_b_;
    Eigen::Matrix6d mass_matrix_;
    double m_{};
    double dt_{};
};

}  // namespace vortex::control

#endif  // DP_ADAPT_BACKS_CONTROLLER_HPP
