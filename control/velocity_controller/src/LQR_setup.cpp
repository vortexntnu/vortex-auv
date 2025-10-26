#include "velocity_controller/LQR_setup.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <vector>
#include <Eigen/Dense>
//#include <drake/common/find_resource.h>
//#include <drake/math/discrete_algebraic_riccati_equation.h>
//#include <drake/math/continuous_algebraic_riccati_equation.h>
//#include <drake/systems/controllers/linear_quadratic_regulator.h>
#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/utilities.hpp"
#include <casadi/casadi.hpp>
#include <lapack.h>

LQRController::LQRController(LQRparameters params,Eigen::Matrix3d inertia_matrix){
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

Eigen::Matrix3d LQRController::calculate_coriolis_matrix(double pitchrate, double yaw_rate, double sway_vel, double heave_vel){
    //Inertia matrix values??
    Eigen::Matrix3d result;
    result<<0.2,-30*sway_vel*0.01,-30*heave_vel*0.01,
            30 * sway_vel*0.01,0,1.629 * pitchrate,
            30 * heave_vel*0.01,1.769 * yaw_rate,0;
    return result;
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
void LQRController::set_matrices(Eigen::Matrix3d inertia_matrix){
    inertia_matrix_inv = inertia_matrix.inverse();
    state_weight_matrix.diagonal() <<q_surge,q_pitch,q_yaw,i_weight,i_weight,i_weight;
    input_weight_matrix.diagonal()<<r_surge,r_pitch,r_yaw;
    return;
}


void LQRController::update_augmented_matrices(Eigen::Matrix3d coriolis_matrix){
    Eigen::Matrix3d system_matrix = inertia_matrix_inv*coriolis_matrix;
    //input_matrix = inertia_matrix_inv;
    augmented_system_matrix <<system_matrix(0,0),system_matrix(0,1),system_matrix(0,2),0,0,0,
                               system_matrix(1,0),system_matrix(1,1),system_matrix(1,2),0,0,0,
                               system_matrix(2,0),system_matrix(2,1),system_matrix(2,2),0,0,0,
                               -1,0,0,0,0,0,
                               0,-1,0,0,0,0,
                               0,0,-1,0,0,0; 
    augmented_input_matrix << inertia_matrix_inv(0,0),inertia_matrix_inv(0,1),inertia_matrix_inv(0,2),0,0,0,
                              inertia_matrix_inv(1,0),inertia_matrix_inv(1,1),inertia_matrix_inv(1,2),0,0,0,
                              inertia_matrix_inv(2,0),inertia_matrix_inv(2,1),inertia_matrix_inv(2,2),0,0,0;
    return;
};
Eigen::Vector<double,6> LQRController::update_error(Guidance_data guidance_values, State states){
    double surge_error = guidance_values.surge - states.surge;
    double pitch_error = ssa(guidance_values.pitch - states.pitch);
    double yaw_error = ssa(guidance_values.yaw - states.yaw);   

    integral_error_surge = anti_windup(i_surge, surge_error, integral_error_surge, surge_windup);
    integral_error_pitch = anti_windup(i_pitch, pitch_error, integral_error_pitch, pitch_windup);
    integral_error_yaw = anti_windup(i_yaw, yaw_error, integral_error_yaw, yaw_windup);

    Eigen::Vector<double,6> state_error= {-surge_error, -pitch_error, -yaw_error, integral_error_surge, integral_error_pitch, integral_error_yaw};
    return state_error;
}
Eigen::Vector<double,3> LQRController::saturate_input(Eigen::Vector<double,3> u){
    double force_x, torque_y, torque_z;
    std::tie(surge_windup, force_x) = saturate(u[0], surge_windup, max_force);
    std::tie(pitch_windup, torque_y) = saturate(u[1], pitch_windup, max_force);
    std::tie(yaw_windup, torque_z) = saturate(u[2], yaw_windup, max_force);
    return {force_x, torque_y, torque_z};
}
Eigen::Vector<double,3> LQRController::calculate_lqr_u(Eigen::Matrix3d coriolis_matrix, State states, Guidance_data guidance_values){
    update_augmented_matrices(coriolis_matrix);
    LQRsolveResult result = solve_k_p(augmented_system_matrix,augmented_input_matrix,state_weight_matrix,input_weight_matrix);
    Eigen::Matrix<double,6,1> state_error = update_error(guidance_values, states);
    Eigen::Vector<double,3> u= saturate_input(- (result.K*state_error));
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


extern "C" {
    // Fortran subroutine for solving symplectic Schur decomposition(double precision version)
void sb02mt_(
    const char* JOBG, const char* JOBL, const char* FACT, const char* UPLO, 
    const int* N, const int* M, double* A, const int* LDA, double* B, const int* LDB,
    double* Q, const int* LDQ, double* R, const int* LDR, double* L, const int* LDL,
    int* IPIV, const int* OUFACT, double* G, const int* LDG,
    int* IWORK, double* DWORK, const int* LDWORK, int* INFO 
);
}


LQRsolveResult LQRController::solve_k_p(Eigen::Matrix<double,6,6> A,Eigen::Matrix<double,6,3> B, Eigen::Matrix<double,6,6> Q,Eigen::Matrix<double,3,3> R){
    //First calculate G with sb02mt_
    char JOBG='G'; //calculate G
    char JOBL='Z'; //L is zero
    char FACT='N'; //unfactored R
    char UPLO='U'; //Upper triangle i think
    const int N=6; //Order of matrices A, Q, G and X(P)
    const int M=3; //Order of matrix R and nuber of columns in B and L(is zero)
    int LDA=N, LDB=M, LDQ=N,LDR=M,LDL=N,LDG=N;

    std::vector<int> IWORK(N);
    int LDWORK=10*N*N; //Upper bounds
    std::vector<double> DWORK(LDWORK);
    std::vector<int> IPIV(N);
    int OUFACT=0; //Output but initialized JIC
    Eigen::Matrix<double,6,6> L=Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,6> G;
    int INFO;
    sb02mt_(&JOBG,&JOBL,&FACT,&UPLO,&N,&M,A.data(),&LDA,B.data(),&LDB,Q.data(),&LDQ,R.data(),&LDR,L.data(),&LDL,IPIV.data(),&OUFACT,G.data(),&LDG,IWORK.data(),DWORK.data(),&LDWORK,&INFO);

    if (INFO!=0){
        //Some Error handling here. Also check that BRB in invertible
    }
    Eigen::Matrix<double,3,6> K;
    Eigen::Matrix<double,3,3> BRB = R+B.transpose()*G*B;
    K=BRB.inverse()*B.transpose()*G*A;

    return LQRsolveResult(K,G);    

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