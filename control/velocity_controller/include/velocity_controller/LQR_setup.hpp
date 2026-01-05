#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "PID_setup.hpp"
#include <Eigen/Dense>
#include "velocity_controller/utilities.hpp"



/*class Guidance_values{
    //Dataclass to store guidance values for LQR controller
    public:
    double surge=0.0;    double pitch=0.0;    double yaw=0.0;
    double integral_surge=0.0;    double integral_pitch=0.0;    double integral_yaw=0.0;
};
*/
class LQRparameters{
    //Dataclass to store LQR parameters
    public:
    double q_surge=0.0;    double q_pitch=0.0;    double q_yaw=0.0;
    double r_surge=0.0;    double r_pitch=0.0;    double r_yaw=0.0;
    double i_surge=0.0;    double i_pitch=0.0;    double i_yaw=0.0;
    double i_weight=0.0;   double max_force=0.0;
};

/*class angle{
    public:
    double phit=0.0;
    double thetat=0.0;
    double psit=0.0;
    
};*/

struct LQRsolveResult{
    Eigen::Matrix<double,3,6> K;
    Eigen::Matrix<double,6,6> P;
    int INFO=0;
    LQRsolveResult(Eigen::Matrix<double,3,6> K,Eigen::Matrix<double,6,6> P, int INFO=0):K(K),P(P),INFO(INFO) {};
};
class LQRController{

    public:
    LQRController(LQRparameters params={0,0,0,0,0,0,0,0,0,0,0},Eigen::Matrix3d inertia_matrix=Eigen::Matrix3d::Identity());
    

    void set_params(LQRparameters params);
    Eigen::Matrix3d calculate_coriolis_matrix(double pitchrate, double yaw_rate, double sway_vel, double heave_vel);
    void set_matrices(Eigen::Matrix3d inertia_matrix);
    void update_augmented_matrices(Eigen::Matrix3d coriolis_matrix);

    //angle quaternion_to_euler_angle(double w, double x, double y, double z);
    //double ssa(double angle);

    std::tuple<double,double> saturate (double value, bool windup, double limit);
    double anti_windup(double ki, double error, double integral_sum, bool windup);  
    Eigen::Vector<double,3> saturate_input(Eigen::Vector<double,3> u);

    Eigen::Vector<double,6> update_error(Guidance_data guidance_values, State states);
    Eigen::Vector<double,3> calculate_lqr_u(State states, Guidance_data guidance_values);
    
    LQRsolveResult solve_k_p(const Eigen::Matrix<double,6,6> &A,const Eigen::Matrix<double,6,3> &B,const Eigen::Matrix<double,6,6> &Q, const Eigen::Matrix<double,3,3> &R);

    //Resets controller
    void reset_controller();

    // VariablesEigen::Matrix3d vector_to_matrix3d(const std::vector<double> &other_matrix)
    const double pi=3.14159265358979323846;
    double integral_error_surge;    double integral_error_pitch;    double integral_error_yaw;
    bool surge_windup;    bool pitch_windup;    bool yaw_windup;
    double q_surge;    double q_pitch;    double q_yaw;
    double r_surge;    double r_pitch;    double r_yaw;
    double i_surge;    double i_pitch;    double i_yaw;
    double i_weight;   double max_force;

    Eigen::Matrix3d inertia_matrix_inv;
    Eigen::Matrix<double,6,6> state_weight_matrix;
    Eigen::Matrix3d input_weight_matrix;
    Eigen::Matrix<double,6,6> augmented_system_matrix;
    Eigen::Matrix<double,6,3> augmented_input_matrix;

    
};

//Extra operations
Eigen::Matrix3d vector_to_matrix3d(const std::vector<double> &other_matrix);
std::vector<double> matrix3d_to_vector(const Eigen::Matrix3d &mat);
std::vector<std::vector<double>> matrix3d_to_vector2d(const Eigen::Matrix3d &mat);
Eigen::Matrix3d vector2d_to_matrix3d(const std::vector<std::vector<double>> &other_matrix);
