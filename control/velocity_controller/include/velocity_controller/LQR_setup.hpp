#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <Eigen/Dense>
#include "velocity_controller/utilities.hpp"

class LQRController{

    public:
    LQRController();
    bool set_matrices(std::vector<double> Q_,std::vector<double> R_,std::vector<double> inertia_matrix, double max_force,std::vector<double> D_low);
    void reset_controller(int nr=0);
    bool calculate_thrust(State states, Guidance_data guidance_values);
    bool set_interval(double interval);
    Eigen::Vector<double,3> get_thrust();
    private:
    Eigen::Matrix<double,8,8> linearize(State states);
    Eigen::Matrix<double,6,6> coriolis(const State& s);


    std::tuple<double,double> saturate (double value, bool windup, double limit);
    double anti_windup(double error, double integral_sum, bool windup);  
    Eigen::Vector<double,3> saturate_input(Eigen::Vector<double,3> u);

    Eigen::Vector<double,8> update_error(const Guidance_data& error, const State& state);
    
    double interval_;
    double integral_error_surge;    double integral_error_pitch;    double integral_error_yaw;
    bool surge_windup;    bool pitch_windup;    bool yaw_windup;
    Eigen::Matrix<double,8,8> Q;    Eigen::Matrix<double,3,3> R;  Eigen::Matrix<double,8,3> B;
    Eigen::Matrix<double,6,6> D; 
    double max_force, mass, Ixx, Iyy,Izz;

    Eigen::Matrix<double,6,6> inertia_matrix_inv;
    Eigen::Matrix<double,6,6> state_weight_matrix;
    Eigen::Matrix3d input_weight_matrix;
    Eigen::Matrix<double,6,6> augmented_system_matrix;
    Eigen::Matrix<double,6,3> augmented_input_matrix;
    Eigen::Vector<double,3> u;

    

    
};

