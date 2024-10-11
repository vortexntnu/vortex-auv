#include "rclcpp/rclcpp.hpp"
#include "../include/dp_controller/main.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <tuple>

using namespace std::chrono_literals;

// Define a global vector to store the previous nu_e for the next iteration
Eigen::VectorXd nu_e_prev_global(6);

double eta_e_prev_global = 0;
double integral_eta_e_global = 0;

// Declare global variable for integral of eta_e
Eigen::VectorXd integral_eta_e_global(6);

// Initialize it to zero
void initialize_global_vector() {
    nu_e_prev_global.setZero();
    integral_eta_e_global.setZero();
}

// Decompose the eta message into a 7x1 vector
std::tuple<Eigen::VectorXd, Eigen::VectorXd> format_eta(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::VectorXd eta(7);
    eta << msg->pose.pose.position.x, 
           msg->pose.pose.position.y, 
           msg->pose.pose.position.z,
           msg->pose.pose.orientation.w,
           msg->pose.pose.orientation.x,
           msg->pose.pose.orientation.y,
           msg->pose.pose.orientation.z;
    
    Eigen::VectorXd nu(6);
    nu << msg->twist.twist.linear.x,
          msg->twist.twist.linear.y,
          msg->twist.twist.linear.z,
          msg->twist.twist.angular.x,
          msg->twist.twist.angular.y,
          msg->twist.twist.angular.z;

    return std::make_tuple(eta, nu);
};

geometry_msgs::msg::Wrench conver_to_wrench(const Eigen::VectorXd& vec)
{
    geometry_msgs::msg::Wrench wrench_msg;

    if (vec.size() != 6) {
        // Handle error for incorrect vector size
        throw std::invalid_argument("Vector size must be 6 to convert to Wrench message.");

    } else {
        wrench_msg.force.x = vec[0];
        wrench_msg.force.y = vec[1];
        wrench_msg.force.z = vec[2];

        wrench_msg.torque.x = vec[3];
        wrench_msg.torque.y = vec[4];
        wrench_msg.torque.z = vec[5];
    }
    return wrench_msg;
}

// Publisher class
SMC_node::SMC_node(): Node("SMC_Controller2")
{
    wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("talha_temp", 10);

    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/nucleus/odom", 10, std::bind(&SMC_node::odometry_callback, this, std::placeholders::_1));
};


// Function to create a skew-symmetric matrix from a 3D vector
Eigen::MatrixXd skew(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d skewM;
    skewM <<     0,  -v(2),  v(1),
               v(2),     0, -v(0),
              -v(1),  v(0),     0;
    
    return skewM;
};

// Function to compute the Crb matrix
Eigen::MatrixXd Crb(const Eigen::VectorXd& v_1, const Eigen::Matrix3d& I_1)
{   
    Eigen::MatrixXd Crb(6, 6);
    Crb.setZero(); 

    int m = 30;

    // Extract the angular velocity part (v_1(3), v_1(4), v_1(5))
    Eigen::Vector3d v(v_1(3), v_1(4), v_1(5));
    Eigen::Vector3d p(0.01, 0, 0.02);

    // Compute the inertia term, only for the angular velocity part of v_1
    Eigen::Vector3d I_v = I_1 * v;

    // Compute skew matrices
    Eigen::Matrix3d Skew_v = skew(v);
    Eigen::Matrix3d Skew_p = skew(p);

    // Fill in the Crb matrix blocks
    Crb.block<3,3>(0,0) = m * Skew_v;
    Crb.block<3,3>(0,3) = -m * Skew_v * Skew_p;
    Crb.block<3,3>(3,0) = m * Skew_p * Skew_v;
    Crb.block<3,3>(3,3) = -skew(I_v);

    return Crb;
};

// Function for the the tranformations matrix 
Eigen::MatrixXd T(const Eigen::VectorXd& nu)
{
    Eigen::MatrixXd T(3,3);
    // Define the T vector
    T << 1, sin(nu[3]) * tan(nu[4]), cos(nu[3]) * tan(nu[4]),
         0, cos(nu[3]), -sin(nu[3]),
         0, sin(nu[3]) / cos(nu[4]), cos(nu[3]) / cos(nu[4]);

    return T;
};

// Function for the Rotation
Eigen::MatrixXd R(const Eigen::VectorXd& nu)
{
    Eigen::MatrixXd R(3,3);

    // Define the R vector
    R << cos(nu[5]) * cos(nu[4]), sin(nu[3]) * sin(nu[5]) * cos(nu[4]) - cos(nu[3]) * sin(nu[5]), cos(nu[3]) * sin(nu[5]) + sin(nu[3]) * sin(nu[4]) * cos(nu[5]),
         cos(nu[5]) * sin(nu[4]), sin(nu[3]) * sin(nu[5]) * sin(nu[4]) + cos(nu[3]) * cos(nu[5]), cos(nu[3]) * sin(nu[5]) * sin(nu[4]) - sin(nu[3]) * cos(nu[5]),
         -sin(nu[5]), sin(nu[3]) * cos(nu[5]), cos(nu[3]) * cos(nu[5]);

    return R;
};

// Function to construct the J matrix
Eigen::MatrixXd J(const Eigen::VectorXd& nu)
{
    Eigen::MatrixXd J(6,6);
    J.setZero();

    // Define the J matrix
    J.block<3,3>(0,0) = R(nu); // R(nu) in the top left
    J.block<3,3>(3,3) = T(nu); // T(nu) in the bottom right

    return J;
};

Eigen::VectorXd g_eta(const Eigen::VectorXd& nu, const Eigen::VectorXd& r_bg, const double W, const double B)
{
    Eigen::VectorXd g(6);
    g << (W-B)*sin(nu(4)),
         -(W-B)*cos(nu(4))*sin(nu(3)),
         -(W-B)*cos(nu(4))*cos(nu(3)),
         


    return g;
};

Eigen::MatrixXd inertia_euler(const Eigen::VectorXd& eta)
{
    Eigen::MatrixXd inertia(6, 6);
    inertia.setZero();

    // Define the inertia matrix
    Eigen::Matrix3d 

    // Define the mass of the vehicle
    double m = 30;

    // Define the position of the center of mass
    Eigen::Vector3d p(0.01, 0, 0.02);

    // Define the rotation matrix
    Eigen::Matrix3d R = Eigen::Quaterniond(eta[3], eta[4], eta[5], eta[6]).toRotationMatrix();

    // Compute the inertia matrix
    inertia.block<3,3>(0,0) = m * Eigen::Matrix3d::Identity();
    inertia.block<3,3>(3,3) = I;

    return inertia;
};


// Function for controller
Eigen::VectorXd tau_PID(const double dt, const Eigen::VectorXd& eta, const Eigen::VectorXd& eta_d)
{
    // Defined the tuning variables
    Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd Ki = Eigen::MatrixXd::Identity(6, 6);

    Kp.diagonal() << 1, 1, 1, 1, 1, 1;
    Kd.diagonal() << 1, 1, 1, 1, 1, 1;
    Ki.diagonal() << 1, 1, 1, 1, 1, 1;

    // Compute the error
    Eigen::VectorXd eta_e = eta_d - eta;

    // Compute the integral of the error
    Eigen::VectorXd integral_eta_e = integral_eta_e_global + eta_e;

    // Compute the derivative of the error
    Eigen::VectorXd eta_e_dot = (eta_e - eta_e_prev_global) / dt;

    // Update the global variables
    integral_eta_e_global = integral_eta_e;

    // Anti-windup mechanism
    double max_integral = 10.0; // Define a maximum value for the integral term
    for (int i = 0; i < integral_eta_e.size(); ++i) {
        if (integral_eta_e[i] > max_integral) {
            integral_eta_e[i] = max_integral;
        } else if (integral_eta_e[i] < -max_integral) {
            integral_eta_e[i] = -max_integral;
        }
    }

    // Compute the control input
    Eigen::VectorXd tau = Kp * eta_e + Kd * eta_e_dot + Ki * integral_eta_e;

    return tau;
}

// Function to compute the control input
Eigen::VectorXd tau(const Eigen::VectorXd& eta, const Eigen::VectorXd& nu, const Eigen::VectorXd& eta_d, const Eigen::VectorXd& nu_d)
{
    

    return tau;
}


Eigen::VectorXd run_u(const nav_msgs::msg::Odometry::SharedPtr msg, Eigen::VectorXd d_eta, Eigen::VectorXd d_nu)
{
    // Define the Mrb matrix
    Eigen::MatrixXd Mrb(6, 6);
    Eigen::MatrixXd Ma(6, 6);
    Eigen::Matrix3d I;  // No need to specify dimensions here; Matrix3d is always 3x3


    Eigen::VectorXd eta, nu;
    std::tie(eta, nu) = format_eta(msg);

    // Define the inertia matrix
    I << 0.4278, 0, 0,
         0, 1.629, 0,
         0, 0, 1.7688;

    // Fill in the values for Mrb
     Mrb << 30, 0, 0, 0, 0.6, 0,
            0, 30, 0, -0.6, 0, 0.3,
            0, 0, 30, 0, -0.3, 0,
            0, -0.6, 0, 0.4278, 0, 0,
            0.6, 0, -0.3, 0, 1.629, 0,
            0, 0.3, 0, 0, 0, 1.7688;

    Ma << 3.6, 0, 0, 0, 0, 0,
            0, 108.2, 0, 0, 0, 0,
            0, 0, 108,2, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 26,8, 0,
            0, 0, 0, 0, 0, 26,8;

        
    // Compute Crb matrix using velocity and inertia
    Eigen::MatrixXd Crb_matrix = Crb(nu, I);
    Eigen::VectorXd U = u(Mrb, eta, nu, d_eta, d_nu, I);

    // Output the results
    std::cout << "Mrb matrix is: \n" << Mrb << std::endl;
    std::cout << "Crb matrix is: \n" << Crb_matrix << std::endl;
    std::cout << "The input given is: \n" << U << std::endl;

    return U;

}

void SMC_node::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{



    geometry_msgs::msg::Wrench wrench_msg;

    // Limit the wrench input to a maximum of 99 and minimum of -99
    // wrench_msg.force.x = std::max(std::min(temp.force.x, 99.0), -99.0);
    // wrench_msg.force.y = std::max(std::min(temp.force.y, 99.0), -99.0);
    // wrench_msg.force.z = std::max(std::min(temp.force.z, 99.0), -99.0);
    // wrench_msg.torque.x = std::max(std::min(temp.torque.x, 99.0), -99.0);
    // wrench_msg.force.y = std::max(std::min(tau, 99.0), -99.0);
    // wrench_msg.torque.z = std::max(std::min(temp.torque.z, 99.0), -99.0);

    RCLCPP_INFO(this->get_logger(), "I heard: [%f, %f, %f, %f, %f, %f, %f]", msg->pose.pose.position.x, 
           msg->pose.pose.position.y, 
           msg->pose.pose.position.z,
           msg->pose.pose.orientation.x,
           msg->pose.pose.orientation.y,
           msg->pose.pose.orientation.z,
           msg->pose.pose.orientation.w);

    RCLCPP_INFO(this->get_logger(), "Publishing Wrench: force(%f, %f, %f), torque(%f, %f, %f)",
                wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z,
                wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z);
    
    wrench_publisher_->publish(wrench_msg);

};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SMC_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
};
