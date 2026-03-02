#ifndef LANDMARK_EGOMOTION__LIB__LANDMARK_TYPEDEFS_HPP_
#define LANDMARK_EGOMOTION__LIB__LANDMARK_TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

struct LandmarkMeasurement {
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
    Eigen::Matrix<double, 6, 6> R;
    double stamp_ = 0.0;
};

struct VoConfig {
    double nis_gate_pose;
    double nis_gate_vel;
    double dropout_timeout;
    double pos_floor;
    double att_floor;
    double vel_floor;
    double vel_alpha;
    double dt_min;
    double dt_max;
    double sw_max_age;
    double sw_huber_deg;
    double sw_gate_deg;
    int sw_window_size;
    int rejects_limit;
    bool use_sw;
};

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 15, 6> Matrix15x6d;
typedef Matrix<double, 6, 15> Matrix6x15d;
}  // namespace Eigen

#endif  // LANDMARK_EGOMOTION__LIB__LANDMARK_TYPEDEFS_HPP_
