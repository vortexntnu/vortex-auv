#ifndef DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_HPP_
#define DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_HPP_

#include "dp_adapt_backs_controller/typedefs.hpp"

class DPAdaptBacksController {
   public:
    explicit DPAdaptBacksController(const dp_types::DPAdaptParams adap_params);

    // @brief Calculate thecontrol input tau
    // @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch,
    // yaw]
    // @param eta_d: 6D vector containing the desired vehicle pose [x, y, z,
    // roll, pitch, yaw]
    // @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
    // @return 6D vector containing the control input tau [X, Y, Z, K, M, N]
    dp_types::Vector6d calculate_tau(const dp_types::Eta& eta,
                                     const dp_types::Eta& eta_d,
                                     const dp_types::Nu& nu);

    // @brief Reset the adaptive parameters
    void reset_adap_param();

    // @brief Reset the disturbance estimate
    void reset_d_est();

    // @brief Set the time step
    // @param dt: Time step
    void set_timeStep(double dt);

   private:
    dp_types::Matrix6d K1_;
    dp_types::Matrix6d K2_;
    dp_types::Vector3d r_b_bg_;
    dp_types::Matrix12d adapt_gain_;
    dp_types::Matrix6d d_gain_;
    dp_types::Vector12d adap_param_;
    dp_types::Vector6d d_est_;
    dp_types::Matrix3d I_b_;
    dp_types::Matrix6d M_;
    double m_;
    double dt_;
};
#endif  // DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_HPP_
