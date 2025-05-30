#ifndef TUKF_MODEL_HPP
#define TUKF_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include "tukf_rsi/typedefs.hpp"


// @brief Tranformation matrix from quaternion orientation
// @param orientation: Quaternion representing the orientation
// @return 3x3 transformation matrix
Eigen::Matrix3d tranfromation_matrix(const Eigen::Quaterniond& orientation);

// @brief Mass inertia system-matrix (6×6)
// @param inertia_vec: Vector containing the inertia parameters
Eigen::Matrix6d M_rb(const Eigen::Vector9d& inertia_vec);

// @brief Added mass system-matrix (6×6)
// @param added_mass: Vector containing the added mass parameters
Eigen::Matrix6d M_a(const Eigen::Vector6d& added_mass);

// @brief Corilos and centripetal forces system-matrix (6×6)
// @param inertia_vec: Vector containing the inertia parameters
// @param angular_velocity: Angular velocity vector
Eigen::Matrix6d C_rb(const Eigen::Vector9d& inertia_vec,
                               const Eigen::Vector3d& angular_velocity);

// @brief added mass Corilos and centripetal forces system-matrix (6×6)
// @param added_mass: Vector containing the added mass parameters
// @param angular_velocity: Angular velocity vector
// @param velocity: Velocity linear vector
Eigen::Matrix6d C_a(const Eigen::Vector6d& added_mass,
                              const Eigen::Vector3d& angular_velocity,
                              const Eigen::Vector3d& velocity);

// @brief Damping system-matrix (6×6)
// @param damping: Vector containing the damping parameters
Eigen::Matrix6d D_linear(const Eigen::Vector6d& damping);

// @brief generalized froces (6×1)
// @param g_eta_params: Vector containing the g_eta parameters (buoyancy terms)
// @param euler_angles: Euler angles (roll, pitch, yaw)
Eigen::Vector6d G_eta(const Eigen::Vector4d& g_eta_params,
                                const Eigen::Vector3d& euler_angles);

// @brief Dynamics function for the AUV
AUVState F_dynamics(const AUVState& state,
                     double dt,
                     const Eigen::Vector3d& control_input);

#endif // TUKF_MODEL_HPP