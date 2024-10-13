#include "../include/dp_controller/main.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <tuple>

using namespace std::chrono_literals;

// Define a global vector to store the previous nu_e for the next iteration
Eigen::VectorXd nu_e_prev_global(6);
Eigen::VectorXd integral_eta_e_global(6);
Eigen::VectorXd eta_e_prev_global(6);
const double m = 30; //Mass of the vehicle

// Initialize it to zero
void initialize_global_vector() {
  nu_e_prev_global.setZero();
  integral_eta_e_global.setZero();
  eta_e_prev_global.setZero();
}

// Decompose the eta message into a 7x1 vector
std::tuple<Eigen::VectorXd, Eigen::VectorXd> format_eta(const nav_msgs::msg::Odometry::SharedPtr msg) {
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

geometry_msgs::msg::Wrench conver_to_wrench(const Eigen::VectorXd &vec) {
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
SMC_node::SMC_node() : Node("SMC_Controller2") {
  wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/wrench_input", 10);

  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/nucleus/odom", 10, std::bind(&SMC_node::odometry_callback, this, std::placeholders::_1));
};


// ________________________________________________________________________
// CPP FUNCTIONS FOR THE MATH
// _______________________________________________________________________

// Function to create a skew-symmetric matrix from a 3D vector
Eigen::MatrixXd skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d skewM;
  skewM << 0, -v(2), v(1),
      v(2), 0, -v(0),
      -v(1), v(0), 0;

  return skewM;
};

// SSA (Smallest signed angle) function
// --------------------------------------------------------------
double ssa(const double a1, const double a2) {
  
  double difference = fmod(a2 - a1, 360);

  // Ensure the difference is within the range [-180, 180]
  if (difference > 180.0)
      difference -= 360.0;
  else if (difference < -180.0)
      difference += 360.0;
  
  return difference;
};

Eigen::Vector3d ssa_vector(const Eigen::Vector3d &vector1, const Eigen::Vector3d &vector2)
{
  double e1 = ssa(vector1(0), vector2(0));
  double e2 = ssa(vector1(1), vector2(1));
  double e3 = ssa(vector1(2), vector2(2));

  Eigen::Vector3d ssa_vector_angle;
  ssa_vector_angle << e1, e2, e3;

  return ssa_vector_angle;
}

Eigen::VectorXd vector_difference(const Eigen::VectorXd &Current, const Eigen::VectorXd &Desired)
{
  Eigen::VectorXd difference(6);

  difference.head<3>() = Current.head<3>() - Desired.head<3>();

  Eigen::Vector3d v1 = Current.tail<3>();
  Eigen::Vector3d v2 = Desired.tail<3>();

  difference.tail<3>() = ssa_vector(v1, v2);

  return difference;
}


// --------------------------------------------------------------


// Function to convert quaternion to Euler angles (roll, pitch, yaw)
Eigen::Vector3d quaternionToEuler(double w, double x, double y, double z) {
    Eigen::Vector3d euler;
    
    // Roll (x-axis rotation)
    euler(0) = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    
    // Pitch (y-axis rotation)
    euler(1) = std::asin(2.0 * (w * y - z * x));
    
    // Yaw (z-axis rotation)
    euler(2) = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    
    return euler;  // Return the Euler angles as a Vector3d (roll, pitch, yaw)
}


// Function to compute the Crb matrix
// ==============================================================
Eigen::MatrixXd Crb(const Eigen::VectorXd &v_1, const Eigen::Matrix3d &I_1) {
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
  Crb.block<3, 3>(0, 0) = m * Skew_v;
  Crb.block<3, 3>(0, 3) = -m * Skew_v * Skew_p;
  Crb.block<3, 3>(3, 0) = m * Skew_p * Skew_v;
  Crb.block<3, 3>(3, 3) = -skew(I_v);

  return Crb;
};

// ADDED MASS CRB
Eigen::MatrixXd Ca(const Eigen::MatrixXd &M, const Eigen::VectorXd &nu) {
  Eigen::MatrixXd Ca(6, 6);
  Ca.setZero();

  // Extract velocity components and evaluate them into Vector3d
  Eigen::Vector3d v1 = nu.head<3>().eval();
  Eigen::Vector3d v2 = nu.tail<3>().eval();

  // Compute terms and evaluate to ensure correct types
  Eigen::Vector3d term1 = (M.block<3,3>(0,0) * v1 + M.block<3,3>(0,3) * v2).eval();
  Eigen::Vector3d term2 = (M.block<3,3>(3,0) * v1 + M.block<3,3>(3,3) * v2).eval();

  // Assign to the Ca matrix using the skew function
  Ca.block<3,3>(3,0) = -skew(term1);
  Ca.block<3,3>(0,3) = -skew(term1);
  Ca.block<3,3>(3,3) = skew(term2);

  return Ca;
};
// ==============================================================


// Constructing the J matrix in the first differential equation
// ==============================================================
// Function for the the transformations matrix
Eigen::MatrixXd T(const Eigen::VectorXd &nu) {
  Eigen::MatrixXd T(3, 3);
  T << 1, sin(nu(3)) * tan(nu(4)), cos(nu(3)) * tan(nu(4)),
       0, cos(nu(3)), -sin(nu(3)),
       0, sin(nu(3)) / cos(nu(4)), cos(nu(3)) / cos(nu(4));

  return T;
};


// Function for the Rotation
Eigen::MatrixXd R(const Eigen::VectorXd &nu) {
  Eigen::MatrixXd R(3, 3);

  // Extract the angles from nu
  double phi = nu(3);
  double theta = nu(4);
  double psi = nu(5);

  // Compute trigonometric functions
  double cphi = cos(phi);
  double sphi = sin(phi);
  double ctheta = cos(theta);
  double stheta = sin(theta);
  double cpsi = cos(psi);
  double spsi = sin(psi);

  // Define the R matrix
  R << ctheta * cpsi,
       sphi * stheta * cpsi - cphi * spsi,
       cphi * stheta * cpsi + sphi * spsi,

       ctheta * spsi,
       sphi * stheta * spsi + cphi * cpsi,
       cphi * stheta * spsi - sphi * cpsi,

       -stheta,
       sphi * ctheta,
       cphi * ctheta;

  return R;
};


// Function to construct the J matrix
Eigen::MatrixXd create_J(const Eigen::VectorXd &nu) {
  Eigen::MatrixXd J(6, 6);
  J.setZero();

  // Define the J matrix
  J.block<3, 3>(0, 0) = R(nu); // R(nu) in the top left
  J.block<3, 3>(3, 3) = T(nu); // T(nu) in the bottom right

  return J;
};
// ==============================================================

Eigen::MatrixXd M_star_computing(const Eigen::MatrixXd &M, const Eigen::VectorXd &nu) {
  Eigen::MatrixXd M_star(6, 6);
  Eigen::MatrixXd J_matrix = create_J(nu);
  Eigen::MatrixXd J_inv = J_matrix.inverse();

  M_star = J_inv.transpose() * M * J_inv;

  return M_star;
};


Eigen::MatrixXd C_star_computing(const Eigen::MatrixXd &C, const Eigen::VectorXd &nu) {
  Eigen::MatrixXd C_star(6, 6);
  Eigen::MatrixXd J = create_J(nu);
  Eigen::MatrixXd J_inv = J.fullPivLu().inverse();

  C_star = J_inv.transpose() * C * J_inv;

  return C_star;
};

// ==============================================================

Eigen::MatrixXd natrual_freq(const Eigen::MatrixXd &B, const Eigen::MatrixXd &D) {
  Eigen::MatrixXd natural_freq(6, 6);
  natural_freq.setZero();
  
  for (int i = 0; i < 6; ++i) {

    double z_2 = D(i,i)*D(i,i);
    double z_4 = z_2*z_2;

    double demon = sqrt(1 - (2*z_2) + sqrt((4*z_4) - (4*z_2) + 2));

    natural_freq(i,i) = (1/demon)*B(i,i);
  }

  return natural_freq;
};


// Function for controller
Eigen::VectorXd tau_PID(const double dt, const Eigen::MatrixXd &M, const Eigen::VectorXd &nu ,const Eigen::VectorXd &eta, const Eigen::VectorXd &eta_d) {
  // Defined the tuning variables
  Eigen::MatrixXd Kp(6, 6);
  Eigen::MatrixXd Kd(6, 6);
  Eigen::MatrixXd Ki(6, 6);

  // Define the bandwitdh and damping ratio
  Eigen::MatrixXd omega_b(6, 6);
  Eigen::MatrixXd omega_n(6, 6);
  Eigen::MatrixXd Z(6, 6);

  omega_b << 0.1, 0, 0, 0, 0, 0,
             0, 0.1, 0, 0, 0, 0,
             0, 0, 0.1, 0, 0, 0,
             0, 0, 0, 0.1, 0, 0,
             0, 0, 0, 0, 0.1, 0,
             0, 0, 0, 0, 0, 0.1;
  
  Z << 0.7, 0, 0, 0, 0, 0,
        0, 0.7, 0, 0, 0, 0,
        0, 0, 0.7, 0, 0, 0,
        0, 0, 0, 0.7, 0, 0,
        0, 0, 0, 0, 0.7, 0,
        0, 0, 0, 0, 0, 0.7;

  omega_n = natrual_freq(omega_b, Z);

  // ___________________________________________________________

  Eigen::MatrixXd M_star_current = M_star_computing(M, nu);

  Kp = M_star_current * omega_n * omega_n;

  Kd = 2 * M_star_current * omega_n * Z;

  Ki = (1.0 / 10.0) * Kp * omega_n;

  // Compute the error
  Eigen::VectorXd eta_e = vector_difference(eta, eta_d);

  // Compute the integral of the error
  Eigen::VectorXd integral_eta_e = integral_eta_e_global + eta_e;

  // Compute the derivative of the error
  Eigen::VectorXd eta_e_dot = vector_difference(eta, eta_e_prev_global) / dt;

  // Update the global variables
  integral_eta_e_global = integral_eta_e;
  eta_e_prev_global = eta_e;

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

Eigen::VectorXd run_u(const nav_msgs::msg::Odometry::SharedPtr msg, Eigen::VectorXd d_eta, Eigen::VectorXd d_nu) {
  // Define the Mrb matrix
  Eigen::MatrixXd Mrb(6, 6);
  Eigen::MatrixXd Ma(6, 6);
  Eigen::MatrixXd damping_ratio(6, 6);
  Eigen::MatrixXd bandwidth_frequency(6, 6);  
  Eigen::Matrix3d I; // No need to specify dimensions here; Matrix3d is always 3x3

  Eigen::VectorXd eta, nu;
  std::tie(eta, nu) = format_eta(msg);

  Eigen::VectorXd eta_temp(6);
  Eigen::VectorXd euler_angles = quaternionToEuler(eta(6), eta(3), eta(4), eta(5));
  // Set the first three components to the position part of eta
  eta_temp.head<3>() = eta.head<3>();

  // Set the last three components to the Euler angles (roll, pitch, yaw)
  eta_temp.tail<3>() = euler_angles;

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
      0, 0, 108.2, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 26.8, 0,
      0, 0, 0, 0, 0, 26.8;

  Eigen::VectorXd U = tau_PID(0.1, Mrb + Ma, nu-d_nu,eta_temp, d_eta);

  std::cout << "The input given is: \n"
            << U << std::endl;

  return U;
}


// Callback function for the odometry message
void SMC_node::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

  // Define desired eta and nu (these should be set according to your control requirements)
  Eigen::VectorXd d_eta(6);
  Eigen::VectorXd d_nu(6);
  d_eta << 0, 0, -0.5, 0, 0, 0;
  d_nu << 0, 0, 0, 0, 0, 0;

  RCLCPP_INFO(this->get_logger(), "I heard: [%f, %f, %f, %f, %f, %f, %f]", msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z,
              msg->pose.pose.orientation.x,
              msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z,
              msg->pose.pose.orientation.w);

  // Call the run_u function with the current odometry message and desired eta and nu
  Eigen::VectorXd tau = run_u(msg, d_eta, d_nu);

  // Convert the resulting tau to a Wrench message
  geometry_msgs::msg::Wrench wrench_msg = conver_to_wrench(tau);

  RCLCPP_INFO(this->get_logger(), "Publishing Wrench: force(%f, %f, %f), torque(%f, %f, %f)",
              wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z,
              wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z);

  wrench_publisher_->publish(wrench_msg);
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  initialize_global_vector();
  auto node = std::make_shared<SMC_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
};
