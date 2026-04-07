#ifndef PID_CONTROLLER_DP__PID_CONTROLLER_UTILS_HPP_
#define PID_CONTROLLER_DP__PID_CONTROLLER_UTILS_HPP_

#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vortex/utils/types.hpp>
#include "pid_controller_dp/typedefs.hpp"
#include "typedefs.hpp"

/**
 * @brief Smallest signed angle (SSA) — wraps an angle to [-π, π].
 * @param angle Angle in radians
 * @return Equivalent angle in [-π, π]
 */
double ssa(double angle);

/**
 * @brief Compute the rotation matrix from body to world frame.
 *
 * REF: Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control",
 * 2021, p.34, eq. 2.72.
 *
 * @param eta Vehicle pose containing the unit quaternion [qw, qx, qy, qz]
 * @return 3×3 rotation matrix R ∈ SO(3)
 */
types::Matrix3d calculate_R_quat(const types::Eta& eta);

/**
 * @brief Compute the 3×3 quaternion kinematic sub-matrix T₃₃.
 *
 * Returns the bottom three rows of the full 4×3 quaternion transformation
 * matrix T, giving the mapping from body angular velocity to
 * d/dt [qx, qy, qz].
 *
 * REF: Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control",
 * 2021, p.35, eq. 2.78.
 *
 * @param eta Vehicle pose containing the unit quaternion
 * @return 3×3 matrix T₃₃
 */
types::Matrix3d calculate_T_quat(const types::Eta& eta);

/**
 * @brief Compute the 6×6 Jacobian J = blockdiag(R, T₃₃).
 *
 * Maps the 6D world-frame error state [Δp; ε_q] to the body frame.
 * T₃₃ maps body angular velocity to the quaternion vector-part rate.
 *
 * @param eta Vehicle pose [x, y, z, qw, qx, qy, qz]
 * @return J_transformation struct holding R and T₃₃ (and their 6×6 product)
 */
types::J_transformation calculate_J(const types::Eta& eta);

/**
 * @brief Compute the inverse of the 6×6 Jacobian: J⁻¹ = blockdiag(Rᵀ, I₃).
 *
 * The exact inverse of T₃₃ is approximated by I₃. When the Jacobian is
 * near-singular the right Moore–Penrose pseudoinverse is used instead.
 *
 * @param eta Vehicle pose [x, y, z, qw, qx, qy, qz]
 * @return 6×6 inverse Jacobian matrix
 */
types::Matrix6d calculate_J_sudo_inv(const types::Eta& eta);

/**
 * @brief Compute the 6D quaternion error state ε̃ = [Δp; ε_q].
 *
 * The scalar part q̃_w is not returned. The sign of the quaternion error is
 * flipped when q̃_w < 0 to enforce the shortest-path convention.
 *
 * @param eta  Actual vehicle pose [x, y, z, qw, qx, qy, qz]
 * @param eta_d Desired vehicle pose [x, y, z, qw, qx, qy, qz]
 * @return Eta struct whose [qx, qy, qz] fields carry ε_q (qw is discarded)
 */
types::Eta error_eta(const types::Eta& eta, const types::Eta& eta_d);

/**
 * @brief Element-wise clamp of a vector to [min_val, max_val].
 * @param values  Input vector
 * @param min_val Lower bound
 * @param max_val Upper bound
 * @return Clamped vector of the same size
 */
Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             double min_val,
                             double max_val);

/**
 * @brief Integrate the body-frame error with anti-windup clamping.
 * @param dt         Time step in seconds
 * @param error_body 6D error in body frame [surge, sway, heave, roll, pitch,
 * yaw]
 * @param integral   Current 6D integral accumulator
 * @return Updated 6D integral after clamping
 */
types::Vector6d anti_windup(const double dt,
                            const types::Vector6d& error_body,
                            const types::Vector6d& integral);

#endif  // PID_CONTROLLER_DP__PID_CONTROLLER_UTILS_HPP_
