#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>



class State{
    //Dataclass to store state values for LQR controller
    public:
    double surge=0.0;    double pitch=0.0;    double yaw=0.0;
    double integral_surge=0.0;    double integral_pitch=0.0;    double integral_yaw=0.0;
};

class Guidance_values{
    //Dataclass to store guidance values for LQR controller
    public:
    double surge=0.0;    double pitch=0.0;    double yaw=0.0;
    double integral_surge=0.0;    double integral_pitch=0.0;    double integral_yaw=0.0;
};

class LQRparameters{
    //Dataclass to store LQR parameters
    public:
    double q_surge=0.0;    double q_pitch=0.0;    double q_yaw=0.0;
    double r_surge=0.0;    double r_pitch=0.0;    double r_yaw=0.0;
    double i_surge=0.0;    double i_pitch=0.0;    double i_yaw=0.0;
    double i_weight=0.0;   double max_force=0.0;
};

class angle{
    public:
    double phit=0.0;
    double thetat=0.0;
    double psit=0.0;
};
class LQRController{

    public:
    LQRController(LQRparameters params,std::vector<double> inertia_matrix);
    angle quaternion_to_euler_angle(double w, double x, double y, double z);
    double ssa(double angle);
    std::tuple<double,double> saturate (double value, bool windup, double limit);
    double anti_windup(double ki, double error, double integral_sum, bool windup);
    std::vector<std::vector<double>> calculate_coriolis_matrix(double pitchrate, double yaw_rate, double sway_vel, double heave_vel);
    void set_params(LQRparameters params);
    void set_matrices(std::vector<double> inertia_matrix);
    void update_augmented_matrices(std::vector <std::vector<double>> coriolis_matrix);
    std::vector<double> update_error(Guidance_values guidance_values, State states);
    std::vector<double> saturate_input(std::vector<double> u);
    std::vector<double> calculate_lqr_u(std::vector<std::vector<double>> coriolis_matrix, State states, Guidance_values guidance_values);
    void reset_controller();

    // Variables
    const double pi=3.14159265358979323846;
    double integral_error_surge;    double integral_error_pitch;    double integral_error_yaw;
    bool surge_windup;    bool pitch_windup;    bool yaw_windup;
    double q_surge;    double q_pitch;    double q_yaw;
    double r_surge;    double r_pitch;    double r_yaw;
    double i_surge;    double i_pitch;    double i_yaw;
    double i_weight;   double max_force;

    std::vector<std::vector<double>> inertia_matrix_inv;
    std::vector<std::vector<double>> state_weight_matrix;
    std::vector<std::vector<double>> input_weight_matrix;
    std::vector<std::vector<double>> augmented_system_matrix;
    std::vector<std::vector<double>> augmented_input_matrix;
};

