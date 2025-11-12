#ifndef LOS_GUIDANCE__LOS_GUIDANCE_HPP_
#define LOS_GUIDANCE__LOS_GUIDANCE_HPP_

#include <eigen3/Eigen/Dense>

namespace LOS {
struct Point {
    double x;
    double y;
    double z;

    Point operator-(const Point& other) const {
        return Point{x - other.x, y - other.y, z - other.z};
    }

    Eigen::Vector3d as_vector() const { return Eigen::Vector3d(x, y, z); }
};

struct CrossTrackError {
    double x_e;
    double y_e;
    double z_e;

    inline static CrossTrackError from_vector(const Eigen::Vector3d& vector) {
        return CrossTrackError{vector.x(), vector.y(), vector.z()};
    }
};

struct Params {
    double lookahead_distance_h;
    double lookahead_distance_v;
    double gamma_h;
    double gamma_v;
    double time_step;
};
};  // namespace LOS

/**
 * @brief Adaptive Line-of-Sight (LOS) guidance algorithm based on slide 113
 * in "Fossen 2024 Lecture on 2D and 3D path-following control".
 */
class AdaptiveLOSGuidance {
   public:
    explicit AdaptiveLOSGuidance(const LOS::Params& params);
    ~AdaptiveLOSGuidance() = default;

    void update_angles(const LOS::Point& prev_point,
                       const LOS::Point& next_point);

    LOS::CrossTrackError calculate_crosstrack_error(
        const LOS::Point& prev_point,
        const LOS::Point& current_position);

    double calculate_psi_d(const double& y_e);

    double calculate_theta_d(const double& z_e);

    void update_adaptive_estimates(
        const LOS::CrossTrackError& crosstrack_error);

   private:
    double time_step_;
    double lookahead_distance_h_;
    double lookahead_distance_v_;
    double gamma_h_;
    double gamma_v_;
    Eigen::Matrix3d rotation_y_;
    Eigen::Matrix3d rotation_z_;
    double pi_h_;
    double pi_v_;
    double beta_c_hat_ = 0.0;
    double alpha_c_hat_ = 0.0;
};

#endif  // LOS_GUIDANCE__LOS_GUIDANCE_HPP_
