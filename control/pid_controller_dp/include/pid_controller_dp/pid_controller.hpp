#ifndef PID_CONTROLLER_DP__PID_CONTROLLER_HPP_
#define PID_CONTROLLER_DP__PID_CONTROLLER_HPP_

#include <spdlog/spdlog.h>
#include "pid_controller_dp/typedefs.hpp"

class PIDController {
   public:
    PIDController();

    // @brief Calculate the control input tau
    // @param eta: struct containing the vehicle pose [position, orientation]
    // @param eta_d:  struct containing the desired vehicle pose [position,
    // orientation]
    // @param nu:  struct containing the vehicle velocity [linear, angular]
    // @param eta_dot_d: struct containing the derivative of the desired vehicle
    // pose [position, orientation]
    // @return 6D vector containing the control input tau [X, Y, Z, K, M, N]
    types::Vector6d calculate_tau(const types::Eta& eta,
                                  const types::Eta& eta_d,
                                  const types::Nu& nu,
                                  const types::Eta& eta_dot_d);

    // @brief Set the proportional gain matrix
    // @param Kp: 6x6 matrix containing the proportional gain matrix
    void set_kp(const types::Matrix6d& Kp);

    // @brief Set the integral gain matrix
    // @param Ki: 6x6 matrix containing the integral gain matrix
    void set_ki(const types::Matrix6d& Ki);

    // @brief Set the derivative gain matrix
    // @param Kd: 6x6 matrix containing the derivative gain matrix
    void set_kd(const types::Matrix6d& Kd);

    // @brief Set the time step
    // @param dt: Time step
    void set_time_step(double dt);

    types::Matrix6d get_kp();
    types::Matrix6d get_ki();
    types::Matrix6d get_kd();

   private:
    types::Matrix6d Kp_;
    types::Matrix6d Ki_;
    types::Matrix6d Kd_;
    types::Vector7d integral_;
    double dt_;
};

#endif  // PID_CONTROLLER_DP__PID_CONTROLLER_HPP_
