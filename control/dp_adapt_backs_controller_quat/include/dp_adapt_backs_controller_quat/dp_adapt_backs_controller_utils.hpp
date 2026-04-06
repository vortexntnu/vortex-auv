#ifndef DP_ADAPT_BACKS_CONTROLLER_QUAT__DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP_
#define DP_ADAPT_BACKS_CONTROLLER_QUAT__DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP_

#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller_quat/typedefs.hpp"
#include "typedefs.hpp"

namespace vortex::control {

/**
 * @brief Calculate the time derivative of the rotation matrix R.
 * @param pose Vehicle pose containing the current orientation quaternion.
 * @param twist Vehicle velocity containing the angular rate omega.
 * @return 3x3 matrix R_dot = R * S(omega).
 */
Eigen::Matrix3d calculate_R_dot(const vortex::utils::types::Pose& pose,
                                const vortex::utils::types::Twist& twist);

/**
 * @brief Build the orientation part of the error-state Jacobian.
 *
 * Computes Q_e = qw_e * I_3 + S(eps_e), the 3x3 block that maps angular
 * velocity to the time derivative of 2*eps_e in the error-state kinematics
 * z_dot_1_ori = Q_e * omega.
 *
 * @param eps_e Vector part of the error quaternion q_e = q_d^* otimes q.
 * @param qw_e  Scalar part of the error quaternion.
 * @return 3x3 matrix Q_e.
 */
Eigen::Matrix3d calculate_Q_e(const Eigen::Vector3d& eps_e, double qw_e);

/**
 * @brief Assemble the 6x6 error-state Jacobian L = diag(R, Q_e).
 *
 * @param R   3x3 rotation matrix from NED to body (from the current pose).
 * @param Q_e 3x3 orientation Jacobian block (from calculate_Q_e).
 * @return 6x6 block-diagonal matrix L.
 */
Eigen::Matrix6d calculate_L(const Eigen::Matrix3d& R,
                            const Eigen::Matrix3d& Q_e);

/**
 * @brief Compute the inverse of the error-state Jacobian L with a singularity
 * guard.
 *
 * Falls back to the Moore-Penrose pseudo-inverse when |det(L)| < tolerance.
 *
 * @param L         Pre-built 6x6 error-state Jacobian.
 * @param tolerance Determinant threshold below which the pseudo-inverse is
 * used.
 * @return 6x6 inverse (or pseudo-inverse) of L.
 */
Eigen::Matrix6d calculate_L_inv(const Eigen::Matrix6d& L,
                                double tolerance = 1e-8);

/**
 * @brief Compute the time derivative of Q_e via error-quaternion kinematics.
 *
 * Derived from q_e_dot using qw_e_dot = -0.5 * eps_e^T * omega and
 * eps_e_dot = 0.5 * Q_e * omega:
 *   Q_e_dot = S(0.5 * Q_e * omega) - 0.5 * (eps_e^T * omega) * I_3
 *
 * @param eps_e Vector part of the error quaternion.
 * @param Q_e   Current Q_e matrix (from calculate_Q_e).
 * @param omega Body-frame angular velocity.
 * @return 3x3 matrix Q_e_dot.
 */
Eigen::Matrix3d calculate_Q_e_dot(const Eigen::Vector3d& eps_e,
                                  const Eigen::Matrix3d& Q_e,
                                  const Eigen::Vector3d& omega);

/**
 * @brief Assemble the time derivative of the error-state Jacobian
 * L_dot = diag(R_dot, Q_e_dot).
 *
 * @param R_dot   3x3 time derivative of the rotation matrix.
 * @param Q_e_dot 3x3 time derivative of Q_e (from calculate_Q_e_dot).
 * @return 6x6 block-diagonal matrix L_dot.
 */
Eigen::Matrix6d calculate_L_dot(const Eigen::Matrix3d& R_dot,
                                const Eigen::Matrix3d& Q_e_dot);

/**
 * @brief Calculate the Coriolis and centripetal matrix.
 * @param mass Vehicle mass.
 * @param r_b_bg Vector from the body-frame origin to the centre of gravity.
 * @param twist Body-frame velocity (linear and angular).
 * @param inertia_matrix_body 3x3 body-frame inertia matrix.
 * @return 6x6 Coriolis matrix C(nu).
 */
Eigen::Matrix6d calculate_coriolis(const double mass,
                                   const Eigen::Vector3d& r_b_bg,
                                   const vortex::utils::types::Twist& twist,
                                   const Eigen::Matrix3d& inertia_matrix_body);

/**
 * @brief Build the damping regressor matrix Y(nu).
 * @param twist Vehicle velocity (one linear + one quadratic damping term per
 * DOF).
 * @return 6x12 regressor matrix Y_v such that Y_v * Theta gives the damping
 * wrench.
 */
Eigen::Matrix6x12d calculate_Y_v(const vortex::utils::types::Twist& twist);

}  // namespace vortex::control

#endif  // DP_ADAPT_BACKS_CONTROLLER_QUAT__DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP_
