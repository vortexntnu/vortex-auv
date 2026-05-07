/**
 * @file types.hpp
 * @brief Defines shared data types used by the LOS guidance algorithms.
 */
#ifndef LOS_GUIDANCE__LIB__TYPES_HPP_
#define LOS_GUIDANCE__LIB__TYPES_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <string>

namespace vortex::guidance::los::types {

/**
 * @brief Represents a 3D point.
 *
 * This struct is used to store waypoint positions and vehicle position data
 * in Cartesian coordinates.
 */
struct Point {
    double x{};
    double y{};
    double z{};

    /**
     * @brief Subtracts another point from this point.
     * @param other The point to subtract.
     * @return Point The resulting difference vector as a point.
     */
    Point operator-(const Point& other) const {
        return Point{x - other.x, y - other.y, z - other.z};
    }

    /**
     * @brief Converts the point to an Eigen 3D vector.
     * @return Eigen::Vector3d The point represented as an Eigen vector.
     */
    Eigen::Vector3d as_vector() const { return Eigen::Vector3d(x, y, z); }

    /**
     * @brief Creates a Point from a ROS point message.
     * @param msg ROS point message.
     * @return Point The converted point.
     */
    static Point point_from_ros(const geometry_msgs::msg::Point& msg) {
        return Point{msg.x, msg.y, msg.z};
    }
};

/**
 * @brief Represents cross-track error in the path-fixed reference frame.
 *
 * The values describe the position error relative to the active path segment
 * in the transformed coordinate frame.
 */
struct CrossTrackError {
    double x_e{};
    double y_e{};
    double z_e{};

    /**
     * @brief Creates a CrossTrackError from an Eigen vector.
     * @param vector Eigen vector containing cross-track error components.
     * @return CrossTrackError The converted cross-track error.
     */
    inline static CrossTrackError from_vector(const Eigen::Vector3d& vector) {
        return CrossTrackError{vector.x(), vector.y(), vector.z()};
    }
};

/**
 * @brief Stores the LOS guidance outputs.
 */
struct Outputs {
    double psi_d{};
    double theta_d{};
};

/**
 * @brief Stores the inputs required by the LOS guidance algorithms.
 */
struct Inputs {
    Point prev_point{};
    Point next_point{};
    Point current_position{};
};

/**
 * @brief Enumerates the available LOS guidance methods.
 */
enum class ActiveLosMethod { PROPORTIONAL, INTEGRAL, ADAPTIVE, VECTOR_FIELD };

/**
 * @brief Converts a string to its corresponding ActiveLosMethod enum value.
 * @param str String representation of the LOS method (case-insensitive variants
 * accepted, e.g. "ADAPTIVE" or "adaptive").
 * @return ActiveLosMethod The corresponding enum value.
 * @throws std::runtime_error if the string does not match a known LOS method.
 */
inline ActiveLosMethod string_to_active_los_method(const std::string& str) {
    if (str == "ADAPTIVE" || str == "adaptive")
        return ActiveLosMethod::ADAPTIVE;
    if (str == "PROPORTIONAL" || str == "proportional")
        return ActiveLosMethod::PROPORTIONAL;
    if (str == "INTEGRAL" || str == "integral")
        return ActiveLosMethod::INTEGRAL;
    if (str == "VECTOR_FIELD" || str == "vector_field")
        return ActiveLosMethod::VECTOR_FIELD;
    throw std::runtime_error("Unknown ActiveLosMethod string: '" + str + "'");
}

/**
 * @brief Converts an integer to its corresponding ActiveLosMethod enum value.
 * @param value Integer representation of the LOS method.
 * @return ActiveLosMethod The corresponding enum value.
 * @throws std::runtime_error if the integer does not match a known LOS method.
 */
inline ActiveLosMethod int_to_active_los_method(int value) {
    switch (value) {
        case static_cast<int>(ActiveLosMethod::ADAPTIVE):
            return ActiveLosMethod::ADAPTIVE;
        case static_cast<int>(ActiveLosMethod::PROPORTIONAL):
            return ActiveLosMethod::PROPORTIONAL;
        case static_cast<int>(ActiveLosMethod::INTEGRAL):
            return ActiveLosMethod::INTEGRAL;
        case static_cast<int>(ActiveLosMethod::VECTOR_FIELD):
            return ActiveLosMethod::VECTOR_FIELD;
        default:
            throw std::runtime_error("Unknown ActiveLosMethod numeric value: " +
                                     std::to_string(value));
    }
}

}  // namespace vortex::guidance::los::types

#endif  // LOS_GUIDANCE__LIB__TYPES_HPP_
