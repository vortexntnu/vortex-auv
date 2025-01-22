#ifndef DP_ADAPT_BACKS_CONTROLLER_HPP
#define DP_ADAPT_BACKS_CONTROLLER_HPP

#include "dp_adapt_backs_controller/typedefs.hpp"
#include "typedefs.hpp"

class DPAdaptBacksController {
   public:
    explicit DPAdaptBacksController();

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

    // @brief Set the K1 matrix
    // @param K1: 6D vector containing the K1 matrix
    void set_k1(const dp_types::Vector6d& K1);

    // @brief Set the K2 matrix
    // @param K2: 6D vector containing the K2 matrix
    void set_k2(const dp_types::Vector6d& K2);

    // @brief Set the r_b_bg vector
    // @param r_b_bg: 3D vector containing the r_b_bg vector
    void set_rbg(const dp_types::Vector3d& r_b_bg);

    // @brief Set the adaptive parameter matrix
    // @param adap_param: 12D vector containing the adaptive parameter matrix
    void set_adap_param(const dp_types::Vector12d& adap_param);

    // @brief Set the d gain matrix
    // @param d_gain: 6D vector containing the d gain matrix
    void set_d_gain(const dp_types::Vector6d& d_gain);

    // @brief Set the inertia matrix
    // @param I_b: 3D vector containing the inertia matrix
    void set_inertia_matrix(const dp_types::Vector3d& I_b);

    // @brief Set the mass inertia matrix
    // @param m: Mass inertia matrix
    void set_mass_inertia_matrix(const dp_types::Matrix6d& M);

    // @brief Set the mass
    // @param m: Mass
    void set_m(double m);

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
#endif
