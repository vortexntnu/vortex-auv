#pragma once
#include <Eigen/Dense>
#include <cstdio>
#include <geometry_msgs/msg/detail/wrench_stamped__struct.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tuple>
#include <vector>
#include "velocity_controller/lib/controller.hpp"
#include "velocity_controller/utilities.hpp"
#include "ct/optcon/lqr/LQR.hpp"

//TODO(henrimha): Make the constructor take in a LQR_params struct and make a LQR_params struct in the header file
//TODO(henrimha): figure out how to hold the matrices in the class, and what to take in as parameters
struct LQR_params{
    std::vector<double> Q;
    std::vector<double> R;
    std::vector<double> inertia_matrix;
    std::vector<double> D_low;
    std::vector<double> D_high;
    double interval;
    LQR_params(std::vector<double> Q_val, std::vector<double> R_val, std::vector<double> inertia_matrix_val, std::vector<double> D_low_val, std::vector<double> D_high_val, double interval_val)
        : Q(Q_val), R(R_val), inertia_matrix(inertia_matrix_val), D_low(D_low_val), D_high(D_high_val), interval(interval_val) {};
    LQR_params() = default;
};
class LQRController : public controller{
   public:
    LQRController(const LQR_params& params, const controller_params& control_params);
    void reset_controller(int nr = 0) override;
    geometry_msgs::msg::WrenchStamped calculate_thrust(const State& state, const State& error_state) override;

   private:
    Eigen::Matrix<double, 8, 8> linearize(const State& states);
    //Eigen::Matrix<double, 6, 6> coriolis(const State& s);

    /*std::tuple<double, double> saturate(double value,
                                        bool windup,
                                        double limit);*/
    /*double anti_windup(double error, double integral_sum, bool windup);*/
    void anti_windup(const State& error_state);
    /*Eigen::Vector<double, 3> saturate_input(Eigen::Vector<double, 3> u);*/

    Eigen::Vector<double, 8> update_error(const State& error_state,
                                          const State& state);
    LQR_params params_;
    double integral_error_surge;
    double integral_error_pitch;
    double integral_error_yaw;
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

    ct::optcon::LQR<8, 3> lqr;
    
    friend class LQRTestAccessor;  // Gir testene tilgang til private medlemmer
};
