#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "pid_controller_dp/eigen_typedefs.hpp"

class PIDController {
    public:
        explicit PIDController();

        Eigen::Vector6d calculate_tau(const Eigen::Vector6d &eta, const Eigen::Vector6d &eta_d, const Eigen::Vector6d &nu, const Eigen::Vector6d &eta_dot_d);

        void setKp(const Eigen::Matrix6d &Kp);

        void setKi(const Eigen::Matrix6d &Ki);

        void setKd(const Eigen::Matrix6d &Kd);

        void setTimeStep(double dt);

    private:
        Eigen::Matrix6d Kp_;
        Eigen::Matrix6d Ki_;
        Eigen::Matrix6d Kd_;
        Eigen::Vector6d integral_;
        double dt;
};

#endif