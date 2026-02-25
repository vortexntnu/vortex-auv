#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "velocity_controller/PID_setup.hpp"
#include "LQR_setup.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vortex_msgs/msg/los_guidance.hpp" 
#include "velocity_controller/NMPC_setup.hpp"
#include "velocity_controller/NMPC_acados.hpp"


class Velocity_node : public rclcpp::Node{
    public:
    Velocity_node();
    //Different initializatin functions
    void get_new_parameters();

    //Timer functions
    void publish_thrust();
    void calc_thrust();

    //Callback functions
    void guidance_callback(const vortex_msgs::msg::LOSGuidance::SharedPtr msg_ptr);
    void killswitch_callback(const std_msgs::msg::Bool::SharedPtr msg_ptr);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr);

    //Publisher instance
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_thrust;
    
    //Timer instance
    rclcpp::TimerBase::SharedPtr timer_calculation;
    rclcpp::TimerBase::SharedPtr timer_publish;
    
    //Subscriber instance
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_Odometry;
    rclcpp::Subscription<vortex_msgs::msg::LOSGuidance>::SharedPtr subscriber_guidance;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_killswitch;

    //Variables for topics

    std::string topic_thrust;
    std::string topic_guidance;
    std::string topic_killswitch;
    std::string topic_odometry;

    //Variables for timers
    int publish_rate;
    double max_force;

    //Stored wrenches values
    vortex_msgs::msg::LOSGuidance reference_in;
    Guidance_data guidance_values;
    Guidance_data current_state;
    geometry_msgs::msg::WrenchStamped thrust_out;


    int controller_type; //1 PID, 2 LQR

    //PID controllers
    PID_controller PID_surge;
    PID_controller PID_yaw;
    PID_controller PID_pitch;

    //LQR Controller
    LQRController lqr_controller;
    //LQRparameters lqr_parameters;
    std::vector<double> Q;
    std::vector<double> R;
    //std::vector<double> Qi;
    //std::vector<double> Ri;
    std::vector<double> inertia_matrix;
    std::vector<double> dampening_matrix_low;
    std::vector<double> dampening_matrix_high;
    //NMPC controller
    NMPC_controller NMPC;
    //NMPC acados
    AuvNMPC NMPC_acados;
    //NMPC parameters
    std::vector<double> Q2;
    std::vector<double> R2;
    //Test
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_reference;

    
};




