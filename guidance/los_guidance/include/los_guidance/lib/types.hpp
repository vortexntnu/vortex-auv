#ifndef TYPES_HPP
#define TYPES_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace vortex::guidance::los::types {

// Point Representation
struct Point {
    double x{};
    double y{};
    double z{};

    // Point Operations
    Point operator-(const Point& other) const {
        return Point{x - other.x, y - other.y, z - other.z};
    }

    // Conversion Functions
    Eigen::Vector3d as_vector() const {
        return Eigen::Vector3d(x, y, z);
    }

    static Point point_from_ros(const geometry_msgs::msg::Point& msg) {
        return Point{msg.x, msg.y, msg.z};
    }
};

// Cross Track Error
struct CrossTrackError {
    double x_e{};
    double y_e{};
    double z_e{};

    inline static CrossTrackError from_vector(const Eigen::Vector3d& vector) {
        return CrossTrackError{vector.x(), vector.y(), vector.z()};
    }
};

// Guidance Outputs
struct Outputs {
    double psi_d{};
    double theta_d{};
};

// Guidance Inputs
struct Inputs {
    Point prev_point{};
    Point next_point{};
    Point current_position{};
};

// Active LOS Method
enum class ActiveLosMethod {
    PROPORTIONAL,
    INTEGRAL,
    ADAPTIVE,
    VECTOR_FIELD
};

}  // namespace vortex::guidance::los::types

#endif  // TYPES_HPP