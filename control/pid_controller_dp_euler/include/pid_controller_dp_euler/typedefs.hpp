/**
 * @file eigen_typedefs.hpp
 * @brief Contains the typedef for a 6x1 Eigen vector and a 6x6 Eigen matrix.
 */

#ifndef VORTEX_EIGEN_TYPEDEFS_H
#define VORTEX_EIGEN_TYPEDEFS_H

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

struct Eta {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    Eta operator-(const Eta& other) const {
        Eta eta;
        eta.x = x - other.x;
        eta.y = y - other.y;
        eta.z = z - other.z;
        eta.roll = roll - other.roll;
        eta.pitch = pitch - other.pitch;
        eta.yaw = yaw - other.yaw;
        return eta;
    }

    Vector6d to_vector() const {
        Vector6d eta;
        eta << x, y, z, roll, pitch, yaw;
        return eta;
    }

    // @brief Make the rotation matrix according to eq. 2.31 in Fossen, 2021
    Matrix3d as_rotation_matrix() const {
        Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;

        return q.toRotationMatrix();
    }

    // @brief Make the transformation matrix according to eq. 2.41 in Fossen,
    // 2021
    Matrix3d as_transformation_matrix() const {
        double cphi = cos(roll);
        double sphi = sin(roll);
        double ctheta = cos(pitch);
        double stheta = sin(pitch);

        if (ctheta == 0) {
            throw std::runtime_error(
                "Division by zero in transformation matrix.");
        }
        Matrix3d T;
        T(0, 0) = 1;
        T(0, 1) = sphi * stheta / ctheta;
        T(0, 2) = cphi * stheta / ctheta;
        T(1, 0) = 0;
        T(1, 1) = cphi;
        T(1, 2) = -sphi;
        T(2, 0) = 0;
        T(2, 1) = sphi / ctheta;
        T(2, 2) = cphi / ctheta;

        return T;
    }
};

struct Nu {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
    double p = 0.0;
    double q = 0.0;
    double r = 0.0;

    Nu operator-(const Nu& other) const {
        Nu nu;
        nu.u = u - other.u;
        nu.v = v - other.v;
        nu.w = w - other.w;
        nu.p = p - other.p;
        nu.q = q - other.q;
        nu.r = r - other.r;
        return nu;
    }

    Vector6d to_vector() const {
        Vector6d nu;
        nu << u, v, w, p, q, r;
        return nu;
    }
};

#endif
