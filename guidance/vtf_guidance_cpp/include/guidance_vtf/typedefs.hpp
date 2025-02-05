#ifndef VORTEX_EIGEN_TYPEDEFS_H
#define VORTEX_EIGEN_TYPEDEFS_H

#include <cmath>
#include <complex>
#include <eigen3/Eigen/Dense>
#include <vector>

typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 3> Matrix6x3d;

struct State_object {
    Vector3d position = Vector3d::Zero();
    Vector3d velocity = Vector3d::Zero();
    Vector3d angular_velocity = Vector3d::Zero();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    Vector3d get_position() const { return position; }
    Vector3d get_orientation() const { return Vector3d(roll, pitch, yaw); }
    Vector3d get_velocity() const { return velocity; }
    Vector3d get_angular_velocity() const { return angular_velocity; }

    Vector6d get_pos_vel() const {
        Vector6d result;
        result.head<3>() = position;
        result.tail<3>() = velocity;
        return result;
    }

    State_object operator-(const State_object& other) const {
        State_object result;
        result.position = this->position - other.position;
        result.velocity = this->velocity - other.velocity;
        result.angular_velocity =
            this->angular_velocity - other.angular_velocity;
        result.roll = ssa(this->roll - other.roll);
        result.pitch = ssa(this->pitch - other.pitch);
        result.yaw = ssa(this->yaw - other.yaw);
        return result;
    }

   private:
    double ssa(double angle) const {
        double angle_ssa = std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
        return angle_ssa;
    };
};

struct Path {
    std::vector<Vector3d> waypoints;
    std::vector<Vector3d> path;

    void generate_G0_path() {
        for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
            Vector3d wp0 = waypoints[i];
            Vector3d wp1 = waypoints[i + 1];
            Vector3d v = wp1 - wp0;

            double l = v.norm();
            double chi_l = std::atan2(v.y(), v.x());
            double gamma_l = -std::atan2(v.z(), v.head<2>().norm());

            straight_line_path_segment(wp0, l, chi_l, gamma_l);
        }
    }

    void generate_G1_path(double r_h, double r_v) {
        Vector3d wp0 = waypoints[0];

        for (size_t i = 0; i + 2 < waypoints.size(); ++i) {
            Vector3d wp_mid = waypoints[i + 1];
            Vector3d wp_next = waypoints[i + 2];

            Vector3d v1 = wp_mid - wp0;
            Vector3d v2 = wp_next - wp_mid;

            double chi_0 = std::atan2(v1.y(), v1.x());
            double gamma_0 = -std::atan2(v1.z(), v1.head<2>().norm());
            double chi_1 = std::atan2(v2.y(), v2.x());
            double gamma_1 = -std::atan2(v2.z(), v2.head<2>().norm());

            double alpha_h = (chi_1 - chi_0 + M_PI) / 2.0;
            double cut_h = r_h / std::tan(alpha_h);
            double tot_cut =
                std::sqrt(cut_h * cut_h + (cut_h * std::sin(gamma_0)) *
                                              (cut_h * std::sin(gamma_0)));
            double l_1 = v1.norm() - tot_cut;

            straight_line_path_segment(wp0, l_1, chi_0, gamma_0);

            // "Center" c around which we do the horizontal arc
            Vector3d wp1 = path.back();
            double chi_diff = ssa(chi_1 - chi_0);
            Vector3d c = wp1 + ((chi_diff / std::abs(chi_diff)) *
                                (R_z(chi_0) * Vector3d(0, r_h, 0)));
            c.z() = waypoints[i + 1].z();

            curved_path_segment(c, r_h, r_v, chi_0, chi_1, gamma_0, gamma_1);

            // Final segment if we are at the penultimate step
            if (i == waypoints.size() - 3) {
                Vector3d wp2 = path.back();
                double l_2 = v2.norm() - tot_cut;
                straight_line_path_segment(wp2, l_2, chi_1, gamma_1);
            }
            wp0 = path.back();
        }
    }

    void straight_line_path_segment(const Vector3d& start,
                                    double length,
                                    double chi_l,
                                    double gamma_l) {
        for (double varpi = 0.0; varpi <= 1.0; varpi += 0.01) {
            Vector3d pt;
            pt.x() = start.x() +
                     length * varpi * std::cos(chi_l) * std::cos(gamma_l);
            pt.y() = start.y() +
                     length * varpi * std::sin(chi_l) * std::cos(gamma_l);
            pt.z() = start.z() - length * varpi * std::sin(gamma_l);
            path.push_back(pt);
        }
    }

    void curved_path_segment(const Vector3d& c,
                             double r_h,
                             double r_v,
                             double chi_0,
                             double chi_1,
                             double gamma_0,
                             double gamma_1) {
        // Like the Python version, sample from varpi=0..1 in steps
        double chi_diff = ssa(chi_1 - chi_0);
        double l_1 = std::abs(r_v * chi_diff * std::sin(gamma_0)) / 2.0;
        double l_2 = std::abs(r_v * chi_diff * std::sin(gamma_1)) / 2.0;

        for (double varpi = 0.0; varpi <= 1.0; varpi += 0.01) {
            double sign_h = std::copysign(1.0, ssa(chi_0 - chi_1));
            double sign_g0 = std::copysign(1.0, gamma_0);
            double sign_g1 = std::copysign(1.0, gamma_1);

            Vector3d pt;
            pt.x() = c.x() - sign_h * r_h * std::sin(chi_0 + varpi * chi_diff);
            pt.y() = c.y() + sign_h * r_h * std::cos(chi_0 + varpi * chi_diff);

            if (varpi < 0.5)
                pt.z() = c.z() - sign_g0 * l_1 * (varpi - 0.5) * 2.0;
            else
                pt.z() = c.z() - sign_g1 * l_2 * (varpi - 0.5) * 2.0;

            path.push_back(pt);
        }
    }

   private:
    double ssa(double angle) const {
        double angle_ssa = std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
        return angle_ssa;
    };

    Matrix3d R_z(double rotation) const {
        Matrix3d R;
        R << std::cos(rotation), -std::sin(rotation), 0, std::sin(rotation),
            std::cos(rotation), 0, 0, 0, 1;
        return R;
    };
};

#endif  // VORTEX_EIGEN_TYPEDEFS_H
