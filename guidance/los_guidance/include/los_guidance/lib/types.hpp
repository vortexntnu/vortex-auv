#ifndef TYPES_HPP
#define TYPES_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>

namespace vortex::guidance::los::types{
    struct Point {
        double x{};
        double y{};
        double z{};

        Point operator-(const Point& other) const {
            return Point{x - other.x, y - other.y, z - other.z};
        }

        Eigen::Vector3d as_vector() const { return Eigen::Vector3d(x, y, z); }
    };

    struct CrossTrackError {
        double x_e{};
        double y_e{};
        double z_e{};

        inline static CrossTrackError from_vector(const Eigen::Vector3d& vector) {
            return CrossTrackError{vector.x(), vector.y(), vector.z()};
        }
    };

    struct Output {
        double psi_d{};
        double theta_d{};
    }; 

    struct Inputs{
        Point prev_point{};
        Point next_point{};
        Point current_position{};
    };

    enum class Active_LOSMethod {
        PROPORTIONAL,
        INTEGRAL,
        ADAPTIVE
    };

} // namespace vortex::guidance::los::types

#endif