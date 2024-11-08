#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "pid_controller_dp/eigen_typedefs.hpp"

class PIDController {
   public:
    explicit PIDController();

    // @brief Calculate the control input tau
    // @param eta: 7D vector containing the vehicle pose [x, y, z, w, x, y, z]
    // @param eta_d: 7D vector containing the desired vehicle pose [x, y, z, w,
    // x, y, z]
    // @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
    // @param eta_dot_d: 7D vector containing the desired vehicle velocity [u,
    // v, w, p, q, r]
    // @return 6D vector containing the control input tau [u, v, w, p, q, r]
    Eigen::Vector6d calculate_tau(const Eigen::Vector7d& eta,
                                  const Eigen::Vector7d& eta_d,
                                  const Eigen::Vector6d& nu,
                                  const Eigen::Vector7d& eta_dot_d);

    // @brief Set the proportional gain matrix
    // @param Kp: 6x6 matrix containing the proportional gain matrix
    void setKp(const Eigen::Matrix6d& Kp);

    // @brief Set the integral gain matrix
    // @param Ki: 6x6 matrix containing the integral gain matrix
    void setKi(const Eigen::Matrix6d& Ki);

    // @brief Set the derivative gain matrix
    // @param Kd: 6x6 matrix containing the derivative gain matrix
    void setKd(const Eigen::Matrix6d& Kd);

    // @brief Set the time step
    // @param dt: Time step
    void setTimeStep(double dt);

   private:
    Eigen::Matrix6d Kp_;
    Eigen::Matrix6d Ki_;
    Eigen::Matrix6d Kd_;
    Eigen::Vector7d integral_;
    double dt;
};

#endif
