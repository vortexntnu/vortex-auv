#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "pid_controller_dp/eigen_typedefs.hpp"

class PIDController {
    public:
        explicit PIDController();

        Eigen::Vector6d calculate_tau();

        void setKp(const Eigen::Matrix6d &Kp);

        void setKi(const Eigen::Matrix6d &Ki);

        void setKd(const Eigen::Matrix6d &Kd);

        void setTimeStep(double dt);

        void setEta(const Eigen::Vector6d &eta);

        void setEtaD(const Eigen::Vector6d &eta_d);

        void setNu(const Eigen::Vector6d &nu);

    private:
        Eigen::Matrix6d Kp_;
        Eigen::Matrix6d Ki_;
        Eigen::Matrix6d Kd_;
        Eigen::Vector6d eta_;
        Eigen::Vector6d eta_d_;
        Eigen::Vector6d nu_;
        Eigen::Vector6d integral_;
        double dt;
};

#endif