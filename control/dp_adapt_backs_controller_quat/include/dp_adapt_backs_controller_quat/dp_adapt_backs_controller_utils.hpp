#ifndef DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP_
#define DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP_

#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller_quat/typedefs.hpp"
#include "typedefs.hpp"

namespace vortex::control {

// @brief Calculate the derivative of the rotation matrix
// @param pose: 7D vector containing the vehicle pose [x, y, z, qw, qx, qy, qz]
// @param twist: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 3x3 derivative of the rotation matrix
Eigen::Matrix3d calculate_R_dot(const vortex::utils::types::Pose& pose,
                                const vortex::utils::types::Twist& twist);

// @brief Calculate the derivative of the transformation matrix
// @param pose: 6D vector containing the vehicle pose [x, y, z, qw, qx, qy, qz]
// @param twist: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 3x3 derivative of the transformation matrix
Eigen::Matrix3d calculate_Q_dot(const vortex::utils::types::Pose& pose,
                                const vortex::utils::types::Twist& twist);

// @brief Calculate the pseudo-inverse of the Jacobian matrix
// @param pose: 7D vector containing the vehicle pose [x, y, z, qw, qx, qy, qz]
// @return 6x6 pseudo-inverse Jacobian matrix
Eigen::Matrix6d calculate_L_inv(const vortex::utils::types::Pose& pose);

// @brief calculate the derivative of the Jacobian matrix
// @param pose: 7D vector containing the vehicle pose [x, y, z, qw, qx, qy, qz]
// @param twist: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
Eigen::Matrix6d calculate_L_dot(const vortex::utils::types::Pose& pose,
                                const vortex::utils::types::Twist& twist);

// @brief Calculate the coriolis matrix
// @param m: mass of the vehicle
// @param r_b_bg: 3D vector of the body frame to the center of gravity
// @param twist: 6D vector containing linear and angular velocity of the vehicle
// @param inertia_matrix_body : 3D matrix containing the inertia matrix
// @return 6x6 coriolis matrix
Eigen::Matrix6d calculate_coriolis(const double mass,
                                   const Eigen::Vector3d& r_b_bg,
                                   const vortex::utils::types::Twist& twist,
                                   const Eigen::Matrix3d& inertia_matrix_body);

// @brief Calculate the damping matrix for the adaptive backstepping controller
// @param twist: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 6x6 damping matrix
Eigen::Matrix6x12d calculate_Y_v(const vortex::utils::types::Twist& twist);

}  // namespace vortex::control

#endif  // DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP_
