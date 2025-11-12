#ifndef TYPES_HPP
#define TYPES_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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

        static Point point_from_ros(
            const geometry_msgs::msg::Point& msg) {
            return Point{msg.x, msg.y, msg.z};
        }


    };

    struct CrossTrackError {
        double x_e{};
        double y_e{};
        double z_e{};

        inline static CrossTrackError from_vector(const Eigen::Vector3d& vector) {
            return CrossTrackError{vector.x(), vector.y(), vector.z()};
        }
    };

    struct Outputs { 
        double psi_d{};
        double theta_d{};
    }; 

    struct Inputs{
        Point prev_point{};
        Point next_point{};
        Point current_position{};
    };

    enum class ActiveLosMethod {
        PROPORTIONAL,       // 0
        INTEGRAL,           // 1
        ADAPTIVE            // 2
    };

} // namespace vortex::guidance::los::types

#endif