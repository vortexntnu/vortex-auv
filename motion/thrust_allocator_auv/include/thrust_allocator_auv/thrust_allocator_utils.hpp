/**
 * @file thrust_allocator_utils.hpp
 * @brief This file contains utility functions for the thruster allocator
 * module.
 */

#ifndef THRUST_ALLOCATOR_AUV__THRUST_ALLOCATOR_UTILS_HPP_
#define THRUST_ALLOCATOR_AUV__THRUST_ALLOCATOR_UTILS_HPP_

#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <ranges>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <vortex_msgs/msg/thruster_forces.hpp>
#include <vortex/utils/types.hpp>

using vortex::utils::types::Vector6d;

/**
 * @brief Check if the matrix has any NaN or INF elements.
 *
 * @tparam Derived The type of the matrix.
 * @param M The matrix to check.
 * @return true if the matrix has any NaN or INF elements, false otherwise.
 */
template <typename Derived>
inline bool is_invalid_matrix(const Eigen::MatrixBase<Derived>& M) {
    bool has_nan = !(M.array() == M.array()).all();
    bool has_inf = M.array().isInf().any();
    return has_nan || has_inf;
}
/**
 * @brief struct containing all configuration parameters for the different solvers
 * 
 * @param extended_thrust_matrix The extended thrust configuration matrix from Fossen 2021 (11.16). 
 * @param min_force Constraint on minimum amount of force from Fossen 2021 (11.38).
 * @param max_force Constraint on maximum amount of force from Fossen 2021 (11.38).
 * @param input_weight_matrix Diagonal matrix with input weights from Fossen 2021 (11.27).
 * @param slack_weight_matrix Diagonal matrix with slack variable weights matrix from Fossen 2021 (11.38).
 * @param beta The extended thrust configuration matrix from Fossen 2021 (11.38).
 */
struct AllocatorConfig {
    
    Eigen::MatrixXd extended_thrust_matrix;

    // Constraints
    Eigen::VectorXd min_force;
    Eigen::VectorXd max_force;

    // Weighting matrices
    Eigen::MatrixXd input_weight_matrix;
    Eigen::MatrixXd slack_weight_matrix;
    double beta;
};

inline Eigen::MatrixXd calculate_thrust_allocation_matrix(
    const Eigen::MatrixXd& thruster_force_direction,
    const Eigen::MatrixXd& thruster_position,
    const Eigen::Vector3d& center_of_mass) {
    // Initialize thrust allocation matrix
    Eigen::MatrixXd thrust_allocation_matrix = Eigen::MatrixXd::Zero(6, 8);

    // Calculate thrust and moment contributions from each thruster
    for (int i = 0; i < thruster_position.cols(); i++) {
        Eigen::Vector3d pos =
            thruster_position.col(i);  // Thrust vector in body frame
        Eigen::Vector3d F =
            thruster_force_direction.col(i);  // Position vector in body frame

        // Calculate position vector relative to the center of mass
        pos -= center_of_mass;

        // Fill in the thrust allocation matrix
        thrust_allocation_matrix.block<3, 1>(0, i) = F;
        thrust_allocation_matrix.block<3, 1>(3, i) = pos.cross(F);
    }

    thrust_allocation_matrix = thrust_allocation_matrix.array();
    return thrust_allocation_matrix;
}

/**
 * @brief Saturates the values of a given Eigen vector between a minimum and
 * maximum value.
 *
 * @param vec The Eigen vector to be saturated.
 * @param min The minimum value to saturate the vector values to.
 * @param max The maximum value to saturate the vector values to.
 * @return True if all vector values are within the given range, false
 * otherwise.
 */
inline bool saturate_vector_values(Eigen::VectorXd& vec,
                                   double min,
                                   double max) {
    bool all_values_in_range = std::ranges::all_of(
        vec, [min, max](double val) { return val >= min && val <= max; });

    std::ranges::transform(vec, vec.begin(), [min, max](double val) {
        return std::min(std::max(val, min), max);
    });

    return all_values_in_range;
}

/**
 * @brief Converts an Eigen VectorXd to a vortex_msgs::msg::ThrusterForces
 * message.
 *
 * @param u The Eigen VectorXd to be converted.
 * @param msg The vortex_msgs::msg::ThrusterForces message to store the
 * converted values.
 * @return The converted vortex_msgs::msg::ThrusterForces message.
 */
inline vortex_msgs::msg::ThrusterForces array_eigen_to_msg(
    const Eigen::VectorXd& u) {
    vortex_msgs::msg::ThrusterForces msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "base_link";
    msg.thrust = std::vector<double>(u.begin(), u.end());
    return msg;
}

/**
 * @brief Converts a 1D array of doubles to a 2D Eigen matrix.
 *
 * @param matrix The 1D array of doubles to be converted.
 * @param rows The number of rows in the resulting Eigen matrix.
 * @param cols The number of columns in the resulting Eigen matrix.
 * @return The resulting Eigen matrix.
 */
inline Eigen::MatrixXd double_array_to_eigen_matrix(
    const std::vector<double>& matrix,
    int rows,
    int cols) {
    return Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic,
                                          Eigen::Dynamic, Eigen::RowMajor>>(
        matrix.data(), rows, cols);
}

inline Eigen::Vector3d double_array_to_eigen_vector3d(
    const std::vector<double>& vector) {
    // Ensure the input vector has exactly three elements
    if (vector.size() != 3) {
        throw std::invalid_argument(
            "Input vector must have exactly 3 elements");
    }

    // Map the vector to Eigen::Vector3d
    return Eigen::Map<const Eigen::Vector3d>(vector.data());
}

inline Vector6d wrench_to_vector(
    const geometry_msgs::msg::WrenchStamped& msg) {
    Vector6d msg_vector{msg.wrench.force.x,  msg.wrench.force.y,
                               msg.wrench.force.z,  msg.wrench.torque.x,
                               msg.wrench.torque.y, msg.wrench.torque.z};

    return msg_vector;
}

#endif  // THRUST_ALLOCATOR_AUV__THRUST_ALLOCATOR_UTILS_HPP_
