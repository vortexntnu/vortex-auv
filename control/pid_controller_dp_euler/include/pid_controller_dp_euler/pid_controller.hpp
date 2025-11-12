#ifndef PID_CONTROLLER_DP_EULER__PID_CONTROLLER_HPP_
#define PID_CONTROLLER_DP_EULER__PID_CONTROLLER_HPP_

#include <pid_controller_dp_euler/typedefs.hpp>

class PIDController {
   public:
    PIDController();

    Vector6d calculate_tau(const Eta& eta,
                           const Eta& eta_d,
                           const Nu& nu,
                           const Eta& eta_dot_d);

    void set_kp(const Matrix6d& Kp);

    void set_ki(const Matrix6d& Ki);

    void set_kd(const Matrix6d& Kd);

    void set_time_step(double dt);

   private:
    Matrix6d Kp_;
    Matrix6d Ki_;
    Matrix6d Kd_;
    Vector6d integral_;
    double dt_;
};

#endif  // PID_CONTROLLER_DP_EULER__PID_CONTROLLER_HPP_
