
#include "velocity_controller/LQR_setup.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <vector>
#include <Eigen/Dense>
#include <sstream>
//#include <drake/common/find_resource.h>
//#include <drake/math/discrete_algebraic_riccati_equation.h>
//#include <drake/math/continuous_algebraic_riccati_equation.h>
//#include <drake/systems/controllers/linear_quadratic_regulator.h>
#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/utilities.hpp"
#include <casadi/casadi.hpp>
//#include <lapack.h>
#include "vortex/utils/math.hpp"
#include "ct/optcon/lqr/LQR.hpp"   
#include "velocity_controller/NMPC_setup.hpp" 



//Eigen::IOFormat fmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
LQRController::LQRController()
{
    interval_ = 0.0;
    integral_error_surge = 0.0;
    integral_error_pitch = 0.0;
    integral_error_yaw = 0.0;
    surge_windup = false;
    pitch_windup = false;
    yaw_windup = false;
    max_force = 0.0;
    mass = 0.0;
    Q.setZero();
    R.setZero();
    B.setZero();
    D_low.setZero();
    D_high.setZero();
    inertia_matrix_inv.setZero();
};
bool LQRController::set_matrices(std::vector<double> Q_,std::vector<double> R_,std::vector<double> inertia_matrix_,double max_force_, std::vector<double> water_r_low,std::vector<double> water_r_high){
    //Possible error handling here to check for size and allowed values.
    if (Q_.size()!=8){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The Q matrix has the wrong amount of elements");
        return 0;
    }
    if(R_.size()!=3){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The R matrix has the wrong amount of elements");
        return 0;
    }
    if(inertia_matrix_.size()!=36){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The M matrix has the wrong amount of elements");
        return 0;
    }
    if(water_r_low.size()!=36||water_r_high.size()!=36){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The D matrix has the wrong amount of elements");
        return 0;
    }
    if (max_force_<0){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The max_force need to be >0");
        return 0;
    }
    max_force=max_force_;
    // Ensure full matrices are zeroed before assigning diagonals
    Q.setZero();
    R.setZero();
    Q.diagonal() = Eigen::Map<Eigen::VectorXd>(Q_.data(), Q_.size());
    R.diagonal() = Eigen::Map<Eigen::VectorXd>(R_.data(), R_.size());
    Eigen::Matrix<double,6,6> inertia_matrix = Eigen::Map<const Eigen::Matrix<double,6,6>>(inertia_matrix_.data(),6,6);
    D_low=Eigen::Map<const Eigen::Matrix<double,6,6>>(water_r_low.data(),6,6);
    D_high=Eigen::Map<const Eigen::Matrix<double,6,6>>(water_r_high.data(),6,6);
    inertia_matrix_inv=inertia_matrix.inverse();
    
    Eigen::Matrix<double, 6,3>B_t=inertia_matrix_inv*(Eigen::Matrix<double,6,3>()<<1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,1,0, 0,0,1).finished();
    B.setZero();
    Eigen::Matrix<double,9,3> B_m;
    B_m.setZero();
    B_m.block<6,3>(0,0)=B_t;
    std::vector<std::vector<int>> swaplines{{1,7},{2,8},{3,4},{4,5}};
    for (long unsigned int i=0;i<swaplines.size();i++){
        B_m.row(swaplines[i][0]).swap(B_m.row(swaplines[i][1]));
    }
    B.block<5,3>(0,0)=B_m.block<5,3>(0,0);   
    integral_error_surge= 0.0;    integral_error_pitch= 0.0;    integral_error_yaw= 0.0;
    surge_windup= false;    pitch_windup= false;    yaw_windup= false; mass=inertia_matrix_[0];
    return 1;
}

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



double LQRController::anti_windup(double error, double integral_sum, bool windup){
    if (!windup){
        integral_sum += error*interval_;
    }
    return integral_sum;
}



Eigen::Matrix<double,8,8> LQRController::linearize(State s){
    //Eigen::Matrix<double,12,12> A;
    Eigen::Matrix<double,6,6> D=Eigen::Matrix<double,6,6>::Zero();

    if (s.surge<100){ //Threshold tbd
        D=-inertia_matrix_inv*D_low;
    }
    else {
        D=-inertia_matrix_inv*D_high;
    }
    Eigen::Matrix<double,6,6> C=coriolis(s);
    /* //Need to decide if i want to learize around 0?
    C(1,5)=-mass*s.surge;
    C(2,4)=mass*s.surge;
    */

    D-=inertia_matrix_inv*C; //To avoid unneccessary allocation
    
    Eigen::Matrix<double,3,3> T=Eigen::Matrix<double,3,3>::Identity();
    /*T<<1,sin(s.yaw)*tan(s.pitch),cos(s.yaw)*tan(s.pitch),
        0,cos(s.yaw),-sin(s.yaw),
        0,sin(s.yaw)/cos(s.pitch),cos(s.yaw)/cos(s.pitch);*/
    Eigen::Matrix<double,9,9> A;
    A.block<6,6>(0,0)=D;
    A.block<3,3>(0,6)=A.block<3,3>(6,0)=A.block<3,3>(6,6)=Eigen::Matrix3d::Zero();
    A.block<3,3>(6,3)=T;
    std::vector<std::vector<int>> swaplines{{1,7},{2,8},{3,4},{4,5}};
    for (long unsigned int i=0;i<swaplines.size();i++){
        A.row(swaplines[i][0]).swap(A.row(swaplines[i][1]));
        A.col(swaplines[i][0]).swap(A.col(swaplines[i][1]));
    }
    
    Eigen::Matrix<double,8,8> ret;
    ret.setZero();
    ret.block<5,5>(0,0)=A.block<5,5>(0,0);
    ret.block<3,3>(5,0)=Eigen::Matrix3d::Identity();
    
    return ret;
};
Eigen::Vector<double,8> LQRController::update_error(const Guidance_data& error, const State& state){
    double surge_error = error.surge;
    double pitch_error = error.pitch;
    double yaw_error = error.yaw;   
    
    integral_error_surge = anti_windup(surge_error, integral_error_surge, surge_windup);
    integral_error_pitch = anti_windup(pitch_error, integral_error_pitch, pitch_windup);
    integral_error_yaw = anti_windup(yaw_error, integral_error_yaw, yaw_windup);
    
    Eigen::Vector<double,8> state_error= {surge_error, pitch_error, yaw_error, -state.pitch_rate, -state.yaw_rate, integral_error_surge, integral_error_pitch, integral_error_yaw};
    return state_error;
}
Eigen::Vector<double,3> LQRController::saturate_input(Eigen::Vector<double,3> u){
    double force_x, torque_y, torque_z;
    std::tie(surge_windup, force_x) = saturate(u[0], surge_windup, max_force);
    std::tie(pitch_windup, torque_y) = saturate(u[1], pitch_windup, max_force);
    std::tie(yaw_windup, torque_z) = saturate(u[2], yaw_windup, max_force);
    return {force_x, torque_y, torque_z};
}
bool LQRController::calculate_thrust(State state, Guidance_data error){
    ct::optcon::LQR<8,3> lqr;
    Eigen::Matrix<double,3,8> K_l;
    
    //TODO: consider making my own solver using eigen so that it does not need controll_toolbox library
    bool INFO= lqr.compute(Q,R,linearize(state),B,K_l,true,false);
    if(INFO==0){
        return false; 
    }
    

    Eigen::Matrix<double,8,1> state_error = update_error(error, state);
    u=saturate_input( (K_l*state_error));
    return true;
}
void LQRController::reset_controller(int nr){
    if(nr==0||nr==1){
        integral_error_surge=0.0;
        surge_windup=false;
    }
    if(nr==0||nr==2){
        integral_error_pitch=0.0;
        pitch_windup=false;
    }
    if(nr==0||nr==3){
        integral_error_yaw=0.0;
        yaw_windup=false;
    }

    return;
}
bool LQRController::set_interval(double interval){
    if(interval<=0)return false;
    interval_=interval;
    return true;
}

Eigen::Vector<double,3> LQRController::get_thrust(){
    return u;
}
//Hjelpefunksjoner for å konvertere mellom std::vector og Eigen::Matrix3d
/*/
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
*/
//TODO: double check the matrices here
Eigen::Matrix<double,6,6> LQRController::coriolis(const State& s)
{
    // Body velocities
    double u  = s.surge;
    double v  = s.sway;
    double w  = s.heave;
    double p  = s.roll_rate;
    double q  = s.pitch_rate;
    double r  = s.yaw_rate;

    // Inertia matrix components — extract from M
    // M = [m*I   -m*S(r_g)]
    //     [m*S(r_g)   I_b  ]
    // For simplicity assuming diagonal M (no off-diagonal inertia/CoG terms):
    double m   = mass;
    double Ixx = 1.0 / inertia_matrix_inv(3,3);
    double Iyy = 1.0 / inertia_matrix_inv(4,4);
    double Izz = 1.0 / inertia_matrix_inv(5,5);

    Eigen::Matrix<double,6,6> C = Eigen::Matrix<double,6,6>::Zero();

    // Standard Fossen rigid-body Coriolis (diagonal inertia, CoG at origin):
    // C_RB = [  0        m*r    -m*q  ]   top-right 3x3
    //        [ -m*r       0      m*p  ]
    //        [  m*q     -m*p      0   ]
    // and
    // C_RB = [    0      -Izz*r   Iyy*q ]   bottom-left 3x3 (transposed of above with inertia)
    //        [  Izz*r      0     -Ixx*p ]
    //        [ -Iyy*q    Ixx*p     0    ]

    // Top-right block (translational-rotational coupling)
    C(0,4) =  m * w;   C(0,5) = -m * v;
    C(1,3) = -m * w;   C(1,5) =  m * u;
    C(2,3) =  m * v;   C(2,4) = -m * u;

    // Bottom-left block (rotational-translational coupling)
    C(3,1) =  m * w;   C(3,2) = -m * v;
    C(4,0) = -m * w;   C(4,2) =  m * u;
    C(5,0) =  m * v;   C(5,1) = -m * u;

    // Bottom-right block (rotational-rotational coupling)
    C(3,4) =  Izz * r;   C(3,5) = -Iyy * q;
    C(4,3) = -Izz * r;   C(4,5) =  Ixx * p;
    C(5,3) =  Iyy * q;   C(5,4) = -Ixx * p;

    return C;
}