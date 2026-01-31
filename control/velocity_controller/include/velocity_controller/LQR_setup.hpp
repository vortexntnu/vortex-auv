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
/*
struct LQRsolveResult{
    Eigen::MatrixXd K;
    Eigen::MatrixXd P;
    int INFO=0;
    LQRsolveResult(Eigen::MatrixXd K=Eigen::MatrixXd::Zero(),Eigen::MatrixXd P=Eigen::MatrixXd::Zero(), int INFO=0):K(K),P(P),INFO(INFO) {};
};*/
class LQRController{

    public:
    LQRController();
    int set_matrices(std::vector<double> Q_,std::vector<double> R_,std::vector<double> inertia_matrix, double max_force,std::vector<double> water_r_low,std::vector<double> water_r_high);
    void reset_controller();
    Eigen::Vector<double,3> calculate_thrust(State states, Guidance_data guidance_values);
    int set_interval(double interval);

    private:
    //void set_params(LQRparameters params);
    //Eigen::Matrix3d calculate_coriolis_matrix(double pitchrate, double yaw_rate, double sway_vel, double heave_vel);
    Eigen::Matrix<double,8,8> linearize(State states);

    //angle quaternion_to_euler_angle(double w, double x, double y, double z);
    //double ssa(double angle);

    std::tuple<double,double> saturate (double value, bool windup, double limit);
    double anti_windup(double error, double integral_sum, bool windup);  
    Eigen::Vector<double,3> saturate_input(Eigen::Vector<double,3> u);

    Eigen::Vector<double,8> update_error(Guidance_data guidance_values, State states);
    
    //LQRsolveResult solve_lqr(const Eigen::MatrixXd &A,const Eigen::MatrixXd &B,const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);

    //Resets controller

    // VariablesEigen::Matrix3d vector_to_matrix3d(const std::vector<double> &other_matrix)
    double interval_;
    double integral_error_surge;    double integral_error_pitch;    double integral_error_yaw;
    bool surge_windup;    bool pitch_windup;    bool yaw_windup;
    Eigen::Matrix<double,8,8> Q;    Eigen::Matrix<double,3,3> R;  Eigen::Matrix<double,8,3> B;
    Eigen::Matrix<double,6,6> D_low; Eigen::Matrix<double,6,6> D_high;
    double max_force; double mass;

    Eigen::Matrix<double,6,6> inertia_matrix_inv;
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
