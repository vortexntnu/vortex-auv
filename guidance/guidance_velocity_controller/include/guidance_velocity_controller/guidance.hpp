#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

extern std::vector<Eigen::Vector3d> g_waypoints;
 
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

        
    inline static CrossTrackError from_vector(const Eigen::Vector3d& v) {
        return CrossTrackError{v.x(), v.y(), v.z()};
    }
};

}  

class LOSGuidance {
public:
    LOSGuidance(
        double lookahead_distance_h_,
        double lookahead_distance_v_,
        double gamma_h_,
        double gamma_v_,
        double time_step_,
        double u_desired_,
        double u_min_,
        double d_scale_);
    ~LOSGuidance() = default;

    void update_angles(const LOS::Point& new_point, const LOS::Point& prev_point);
    void cross_track_error(const LOS::Point& current_position, const LOS::Point& prev_point);
    double desired_surge_speed(const LOS::Point& destination , const LOS::Point& current_position) ;
    double desired_heading();
    double desired_pitch();
    void update_adaptive_estimates();

    Eigen::Vector3d get_outputs() const;
    LOS::CrossTrackError& cross_track();

private:

    double lookahead_distance_h;
    double lookahead_distance_v;
    double gamma_h;
    double gamma_v;
    double time_step;
    double u_desired;
    double u_min;
    double d_scale;

    Eigen::Matrix3d rotation_y_;
    Eigen::Matrix3d rotation_z_;

    LOS::CrossTrackError cte;

    double pi_h_;
    double pi_v_;

    double beta_c_hat_ = 0.0;
    double alpha_c_hat_ = 0.0;
};
