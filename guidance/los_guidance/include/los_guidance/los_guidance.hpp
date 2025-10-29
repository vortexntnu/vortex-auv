#ifndef LOS_GUIDANCE_HPP
#define LOS_GUIDANCE_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>

namespace vortex::guidance {

    namespace LOS {

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

        struct Params {
            double lookahead_distance_h{};
            double lookahead_distance_v{};
            double gamma_h{};
            double gamma_v{};
            double time_step{};
        };

    }  // namespace LOS

/**
 * @brief Adaptive Line-of-Sight (LOS) guidance algorithm based on slide 113
 * in "Fossen 2024 Lecture on 2D and 3D path-following control".
 */

    class AdaptiveLOSGuidance {
    public:
        AdaptiveLOSGuidance(const LOS::Params& params);
        ~AdaptiveLOSGuidance() = default;

        void update_angles(const LOS::Point& prev_point,
                        const LOS::Point& next_point);

        LOS::CrossTrackError calculate_crosstrack_error(
            const LOS::Point& prev_point,
            const LOS::Point& current_position) const;

        double calculate_psi_d(const double& y_e) const;

        double calculate_theta_d(const double& z_e) const;

        void update_adaptive_estimates(
            const LOS::CrossTrackError& crosstrack_error);

    private:
        LOS::Params params_;
        Eigen::Matrix3d rotation_y_ = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d rotation_z_ = Eigen::Matrix3d::Zero();
        double pi_h_{};
        double pi_v_{};
        double beta_c_hat_{};
        double alpha_c_hat_{};
    };

// ---------------- Proportional LOS Guidance Implementation ----------------

/* 
    * Proportional Line-of-Sight (LOS) guidance algorithm based on page 356
    * form "The handbook of marine craft hydrodynamic and motion controll".
*/
    class ProportionalLOSGuidance {
    public:
        ProportionalLOSGuidance(const LOS::Params& params);
        ~ProportionalLOSGuidance() = default;

        void update_angles(const LOS::Point& prev_point,
                        const LOS::Point& next_point);

        LOS::CrossTrackError calculate_crosstrack_error(
            const LOS::Point& prev_point,
            const LOS::Point& current_position) const;

        double calculate_psi_d(const double& y_e) const;
        double calculate_theta_d(const double& z_e) const;

    private:
        LOS::Params params_;
        Eigen::Matrix3d rotation_y_{Eigen::Matrix3d::Zero()};
        Eigen::Matrix3d rotation_z_{Eigen::Matrix3d::Zero()};
        double pi_h_{}; 
        double pi_v_{};  
    };

}  // namespace vortex::guidance

#endif  // LOS_GUIDANCE_HPP