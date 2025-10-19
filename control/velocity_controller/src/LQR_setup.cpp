#include "velocity_controller/LQR_setup.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <vector>
#include <Eigen/Dense>
#include <drake/common/find_resource.h>
#include <drake/math/discrete_algebraic_riccati_equation.h>
#include <drake/math/continuous_algebraic_riccati_equation.h>
#include <drake/systems/controllers/linear_quadratic_regulator.h>
#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/utilities.hpp"


LQRController::LQRController(LQRparameters params,std::vector<double> inertia_matrix){
    set_params(params);
    set_matrices(inertia_matrix);
};


/*angle LQRController::quaternion_to_euler_angle(double w, double x, double y, double z){
    double ysqr = y * y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    double phi = std::atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double theta = std::asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);
    double psi = std::atan2(t3, t4);

    return {phi, theta, psi};
};*/

double LQRController::ssa(double angle){
    return std::fmod(angle+pi, 2*pi)-pi;
};

//Can be optimized
std::tuple<double,double> LQRController::saturate (double value, bool windup, double limit){
    if (abs(value) > limit){
        windup=true;
        value = limit * (value/abs(value));
    }
    else {
        windup=false;
    }
    return {windup,value};
}



double LQRController::anti_windup(double ki, double error, double integral_sum, bool windup){
    if (!windup){
        integral_sum += error * ki;
    }
    return integral_sum;
}

std::vector<std::vector<double>> LQRController::calculate_coriolis_matrix(double pitchrate, double yaw_rate, double sway_vel, double heave_vel){
    //Inertia matrix values??
    return {{0.2,-30*sway_vel*0.01,-30*heave_vel*0.01},
            {30 * sway_vel*0.01,0,1.629 * pitchrate},
            {30 * heave_vel*0.01,1.769 * yaw_rate,0}};
}


void LQRController::set_params(LQRparameters params){
    //set LQR parameters
    integral_error_surge= 0.0;    integral_error_pitch= 0.0;    integral_error_yaw= 0.0;
    surge_windup= false;    pitch_windup= false;    yaw_windup= false;
    q_surge= params.q_surge;    q_pitch= params.q_pitch;    q_yaw= params.q_yaw;
    r_surge= params.r_surge;    r_pitch= params.r_pitch;    r_yaw= params.r_yaw;
    i_surge= params.i_surge;    i_pitch= params.i_pitch;    i_yaw= params.i_yaw;
    i_weight= params.i_weight;   max_force= params.max_force;
    return;

}
void LQRController::set_matrices(std::vector<double> inertia_matrix){
    Eigen::Matrix3d mat= vector_to_matrix3d(inertia_matrix);
    inertia_matrix_inv = matrix3d_to_vector2d(mat.inverse());
    state_weight_matrix = {{q_surge,0,0,0,0,0},
                           {0,q_pitch,0,0,0,0},
                           {0,0,q_yaw,0,0,0},
                           {0,0,0,i_weight,0,0},
                           {0,0,0,0,i_weight,0},
                           {0,0,0,0,0,i_weight}};
    input_weight_matrix = {{r_surge,0,0},
                           {0,r_pitch,0},
                           {0,0,r_yaw}};

    return;
}


void LQRController::update_augmented_matrices(std::vector <std::vector<double>> coriolis_matrix){
    std::vector<std::vector<double>> system_matrix = matrix3d_to_vector2d(vector2d_to_matrix3d(inertia_matrix_inv) * vector2d_to_matrix3d(coriolis_matrix));
    //input_matrix = inertia_matrix_inv;
    augmented_system_matrix = {{system_matrix[0][0],system_matrix[0][1],system_matrix[0][2],0,0,0},
                               {system_matrix[1][0],system_matrix[1][1],system_matrix[1][2],0,0,0},
                               {system_matrix[2][0],system_matrix[2][1],system_matrix[2][2],0,0,0},
                               {-1,0,0,0,0,0},
                               {0,-1,0,0,0,0},
                               {0,0,-1,0,0,0}}; //Skal det være -1 her?
    augmented_input_matrix = {{inertia_matrix_inv[0][0],inertia_matrix_inv[0][1],inertia_matrix_inv[0][2],0,0,0},
                              {inertia_matrix_inv[1][0],inertia_matrix_inv[1][1],inertia_matrix_inv[1][2],0,0,0},
                              {inertia_matrix_inv[2][0],inertia_matrix_inv[2][1],inertia_matrix_inv[2][2],0,0,0}};

};
std::vector<double> LQRController::update_error(Guidance_data guidance_values, State states){
    double surge_error = guidance_values.surge - states.surge;
    double pitch_error = ssa(guidance_values.pitch - states.pitch);
    double yaw_error = ssa(guidance_values.yaw - states.yaw);   

    integral_error_surge = anti_windup(i_surge, surge_error, integral_error_surge, surge_windup);
    integral_error_pitch = anti_windup(i_pitch, pitch_error, integral_error_pitch, pitch_windup);
    integral_error_yaw = anti_windup(i_yaw, yaw_error, integral_error_yaw, yaw_windup);

    std::vector<double> state_error= {-surge_error, -pitch_error, -yaw_error, integral_error_surge, integral_error_pitch, integral_error_yaw};
    return state_error;
}
std::vector<double> LQRController::saturate_input(std::vector<double> u){
    double force_x, torque_y, torque_z;
    std::tie(surge_windup, force_x) = saturate(u[0], surge_windup, max_force);
    std::tie(pitch_windup, torque_y) = saturate(u[1], pitch_windup, max_force);
    std::tie(yaw_windup, torque_z) = saturate(u[2], yaw_windup, max_force);
    return {force_x, torque_y, torque_z};
}
std::vector<double> LQRController::calculate_lqr_u(std::vector<std::vector<double>> coriolis_matrix, State states, Guidance_data guidance_values){
    update_augmented_matrices(coriolis_matrix);
    auto result = drake::systems::controllers::LinearQuadraticRegulator(
        vector2d_to_matrix3d(augmented_system_matrix),
        vector2d_to_matrix3d(augmented_input_matrix),
        vector2d_to_matrix3d(state_weight_matrix),
        vector2d_to_matrix3d(input_weight_matrix));
    std::vector<double> state_error = update_error(guidance_values, states);
    std::vector<double> u= saturate_input(matrix3d_to_vector(- (result.K * vector_to_matrix3d(state_error))));
    return u;
}
void LQRController::reset_controller(){
    integral_error_surge=0.0;
    integral_error_pitch=0.0;
    integral_error_yaw=0.0;

    surge_windup=false;
    pitch_windup=false;
    yaw_windup=false;
    return;
}



//Hjelpefunksjoner for å konvertere mellom std::vector og Eigen::Matrix3d
Eigen::Matrix3d vector_to_matrix3d(const std::vector<double> &other_matrix){
    Eigen::Matrix3d mat;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat(i, j) = other_matrix[i * 3 + j];
    return mat;
}
std::vector<double> matrix3d_to_vector(const Eigen::Matrix3d &mat){
    std::vector<double> other_matrix(9);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            other_matrix[i * 3 + j] = mat(i, j);
    return other_matrix;
}

std::vector<std::vector<double>> matrix3d_to_vector2d(const Eigen::Matrix3d &mat){
    std::vector<std::vector<double>> other_matrix(3, std::vector<double>(3));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            other_matrix[i][j] = mat(i, j);
    return other_matrix;
}

Eigen::Matrix3d vector2d_to_matrix3d(const std::vector<std::vector<double>> &other_matrix){
    Eigen::Matrix3d mat;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat(i, j) = other_matrix[i][j];
    return mat;
}