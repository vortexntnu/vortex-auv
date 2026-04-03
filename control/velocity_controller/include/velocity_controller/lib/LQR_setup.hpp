#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/msg/detail/wrench_stamped__struct.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tuple>
#include <vector>
#include "velocity_controller/lib/controller.hpp"
#include "velocity_controller/utilities.hpp"
//TODO(henrimha): Make the constructor take in a LQR_params struct and make a LQR_params struct in the header file
//TODO(henrimha): figure out how to hold the matrices in the class, and what to take in as parameters
struct LQR_params{
    std::vector<double> Q;
    std::vector<double> R;
    std::vector<double> inertia_matrix;
    double max_force;
    std::vector<double> D_low;
    std::vector<double> D_high;
    double interval;
};
class LQRController : public controller{
   public:
    LQRController(LQR_params params);
    void reset_controller(int nr = 0) override;
    geometry_msgs::msg::WrenchStamped calculate_thrust(State state, State error_state) override;

   private:
    Eigen::Matrix<double, 8, 8> linearize(State states);
    Eigen::Matrix<double, 6, 6> coriolis(const State& s);

    std::tuple<double, double> saturate(double value,
                                        bool windup,
                                        double limit);
    double anti_windup(double error, double integral_sum, bool windup);
    Eigen::Vector<double, 3> saturate_input(Eigen::Vector<double, 3> u);

    Eigen::Vector<double, 8> update_error(const State& error_state,
                                          const State& state);
    LQR_params params_;
    double integral_error_surge;
    double integral_error_pitch;
    double integral_error_yaw;
    bool surge_windup;
    bool pitch_windup;
    bool yaw_windup;
    Eigen::Matrix<double, 8, 8> Q;
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 8, 3> B;
    Eigen::Matrix<double, 6, 6> D;
    double mass, Ixx, Iyy, Izz;

    Eigen::Matrix<double, 6, 6> inertia_matrix_inv;
    Eigen::Matrix<double, 6, 6> state_weight_matrix;
    Eigen::Matrix3d input_weight_matrix;
    Eigen::Matrix<double, 6, 6> augmented_system_matrix;
    Eigen::Matrix<double, 6, 3> augmented_input_matrix;
};
