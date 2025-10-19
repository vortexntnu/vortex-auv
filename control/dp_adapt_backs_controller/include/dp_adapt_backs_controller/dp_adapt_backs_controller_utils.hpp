#ifndef DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP
#define DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP

#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller/typedefs.hpp"
#include "typedefs.hpp"

namespace vortex::control {

// @brief Calculate the derivative of the rotation matrix
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 3x3 derivative of the rotation matrix
Eigen::Matrix3d calculate_R_dot(const vortex::utils::types::Eta& eta,
                                const vortex::utils::types::Nu& nu);

// @brief Calculate the derivative of the transformation matrix
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 3x3 derivative of the transformation matrix
Eigen::Matrix3d calculate_T_dot(const vortex::utils::types::Eta& eta,
                                const vortex::utils::types::Nu& nu);

// @brief Calculate the pseudo-inverse of the Jacobian matrix
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @return 6x6 pseudo-inverse Jacobian matrix
Eigen::Matrix6d calculate_J_inv(const vortex::utils::types::Eta& eta);

// @brief calculate the derivative of the Jacobian matrix
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
Eigen::Matrix6d calculate_J_dot(const vortex::utils::types::Eta& eta,
                                const vortex::utils::types::Nu& nu);

// @brief Calculate the coriolis matrix
// @param m: mass of the vehicle
// @param r_b_bg: 3D vector of the body frame to the center of gravity
// @param nu_2: 3D vector containing angular velocity of the vehicle
// @param I_b : 3D matrix containing the inertia matrix
// @return 6x6 coriolis matrix
Eigen::Matrix6d calculate_coriolis(const double mass,
                                   const Eigen::Vector3d& r_b_bg,
                                   const vortex::utils::types::Nu& nu,
                                   const Eigen::Matrix3d& I_b);

// @brief Calculate the damping matrix for the adaptive backstepping controller
// @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 6x6 damping matrix
Eigen::Matrix6x12d calculate_Y_v(const vortex::utils::types::Nu& nu);

}  // namespace vortex::control

#endif
