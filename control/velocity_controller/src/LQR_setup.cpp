
#include "velocity_controller/LQR_setup.hpp"
#include "rclcpp/rclcpp.hpp"
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



Eigen::IOFormat fmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
LQRController::LQRController()
{
    
};
int LQRController::set_matrices(std::vector<double> Q_,std::vector<double> R_,std::vector<double> inertia_matrix_,double max_force_, std::vector<double> water_r_low,std::vector<double> water_r_high){
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
    max_force=max_force_;
    Q.diagonal()=Eigen::Map<Eigen::VectorXd>(Q_.data(),Q_.size());
    R.diagonal()=Eigen::Map<Eigen::VectorXd>(R_.data(),R_.size());
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

/*double LQRController::ssa(double angle){
    return std::fmod(angle+pi, 2*pi)-pi;
};*/

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

/*Eigen::Matrix3d LQRController::calculate_coriolis_matrix(double pitchrate, double yaw_rate, double sway_vel, double heave_vel){
    //Inertia matrix values??
    Eigen::Matrix3d result;
    result<<0.2,-30*sway_vel*0.01,-30*heave_vel*0.01,
            30 * sway_vel*0.01,0,1.629 * pitchrate,
            30 * heave_vel*0.01,1.769 * yaw_rate,0;
    return result;
}*/



Eigen::Matrix<double,8,8> LQRController::linearize(State s){
    //Eigen::Matrix<double,12,12> A;
    Eigen::Matrix<double,6,6> D;

    if (s.surge<100){ //Threshold tbd
        D=-inertia_matrix_inv*D_low;
    }
    else {
        D=-inertia_matrix_inv*D_high;
    }
    Eigen::Matrix<double,6,6> C;
    C.setZero(); //Unødvendig kanskje
    C(1,5)=-mass*s.surge;
    C(2,4)=mass*s.surge;
    D-=inertia_matrix_inv*C; //To avoid unneccessary allocation
    /*Eigen::Matrix<double,3,6> T(1.0,sin(s.psi)*tan(s.theta),cos(s.psi)*tan(s.theta),s.pitch*cos(s.psi)*tan(s.theta)-s.yaw*sin(s.psi)*tan(s.theta),(s.pitch*sin(s.psi)+s.yaw*cos(s.psi))/(cos(s.theta)*cos(s.theta)),
                                0,cos(s.psi),-sin(s.psi),s.yaw*sin(s.psi)+s.pitch*cos(s.psi),0,0,
                                0,sin(s.psi)*1/cos(s.theta),cos(s.psi)/cos(s.theta),s.sway*cos(s.psi)/cos(s.theta)-s.pitch*sin(s.psi)/cos(s.theta),(s.yaw*sin(s.psi)+s.pitch*cos(s.psi)*sin(s.theta)/(cos(s.theta)*cos(s.theta))));
*/
    Eigen::Matrix<double,3,3> T;
    T<<1,sin(s.yaw)*tan(s.pitch),cos(s.yaw)*tan(s.pitch),
        0,cos(s.yaw),-sin(s.yaw),
        0,sin(s.yaw)/cos(s.pitch),cos(s.yaw)/cos(s.pitch);
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
    //legge inn integral state #TODO
    ret.block<3,3>(5,0)=-Eigen::Matrix3d::Identity();
    
    return ret;
};
Eigen::Vector<double,8> LQRController::update_error(Guidance_data guidance_values, State states){
    double surge_error = guidance_values.surge - states.surge;
    double pitch_error = vortex::utils::math::ssa(guidance_values.pitch - states.pitch);
    double yaw_error = vortex::utils::math::ssa(guidance_values.yaw - states.yaw);   
    
    integral_error_surge = anti_windup(surge_error, integral_error_surge, surge_windup);
    integral_error_pitch = anti_windup(pitch_error, integral_error_pitch, pitch_windup);
    integral_error_yaw = anti_windup(yaw_error, integral_error_yaw, yaw_windup);
    
    Eigen::Vector<double,8> state_error= {-surge_error, -pitch_error, -yaw_error, -states.pitch_rate, -states.yaw_rate, integral_error_surge, integral_error_pitch, integral_error_yaw};
    return state_error;
}
Eigen::Vector<double,3> LQRController::saturate_input(Eigen::Vector<double,3> u){
    double force_x, torque_y, torque_z;
    std::tie(surge_windup, force_x) = saturate(u[0], surge_windup, max_force);
    std::tie(pitch_windup, torque_y) = saturate(u[1], pitch_windup, max_force);
    std::tie(yaw_windup, torque_z) = saturate(u[2], yaw_windup, max_force);
    return {force_x, torque_y, torque_z};
}
Eigen::Vector<double,3> LQRController::calculate_thrust(State state, Guidance_data guidance_values){
    ct::optcon::LQR<8,3> lqr;
    Eigen::Matrix<double,3,8> K_l;
    bool INFO= lqr.compute(Q,R,linearize(state),B,K_l,true,false);
    if(INFO==0){
        return {9999,9999,9999}; //Need to fix
    }
    /*
    Eigen::Matrix<double,3,6> K;
    K.block<3,3>(0,0)=K_l.block<3,3>(0,0);
    K.block<3,3>(0,3)=K_l.block<3,3>(0,5);
    */

    Eigen::Matrix<double,8,1> state_error = update_error(guidance_values, state);
    return saturate_input(- (K_l*state_error));
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
int LQRController::set_interval(double interval){
    interval_=interval;
    return 1;
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
void sb02md_( const char* DICO, const char* HINV, const char* UPLO, const char* SCAL, const char* SORT, const int*  N, double* A, const int*  LDA, double* G,
                      const int*  LDG, double* Q, const int* LDQ, const double* RCOND, double* WR, double* WI, double*  S, const int*  LDS, double* U, const int*  LDU,
                       int* IWORK, double* DWORK, const int*  LDWORK, int* BWORK, const int* INFO 
                    );
}

/*
LQRsolveResult LQRController::solve_lqr(const Eigen::MatrixXd &A,const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R){

    const int N=A.rows();
    const int M=B.cols();
    if (A.cols()!=N||B.rows()!=N||R.rows()!=M||R.cols()!=M||Q.rows()!=N||Q.cols()!=N){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The dimensions of the matrices for solve_lqr are wrong");
        return LQRsolveResult{};
    }

    Eigen::Matrix<double,6,6> A_copy=A, Q_copy=Q;
    Eigen::Matrix<double,6,3> B_copy=B; Eigen::Matrix<double,3,3> R_copy=R;
    
    //First calculate G with sb02mt_
    //calculate G, L is zero, unfactored R, Upper triangle i think
    char JOBG='G',JOBL='Z',FACT='N',UPLO='U';
    //Order of matrices A, Q, G and X(P), Order of matrix R and nuber of columns in B and L(is zero)
    //Dimensions of matrices
    int LDA=N, LDB=N, LDQ=N,LDR=M,LDL=N,LDG=N;
    std::vector<int> IWORK(8*N),IPIV(N);
    //Upper bounds Output but initialized JIC output placeholder
    int LDWORK=20*N*N,OUFACT=0,INFO=0; 
    std::vector<double> DWORK(LDWORK);
    Eigen::Matrix<double,6,6> L=Eigen::Matrix<double,6,6>::Zero(), G=L;
    {
      double detA = A_copy.determinant();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling sb02md_: det(A)=%.6g, A(0,0)=%.6g, G(0,0)=%.6g", detA, A_copy(0,0), G(0,0));
      Eigen::EigenSolver<Eigen::Matrix<double,6,6>> es(A_copy);
      for (int i=0;i<6;++i){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "eigA[%d]=% .6g%+.6gi", i, es.eigenvalues()[i].real(), es.eigenvalues()[i].imag());
      }
    }
    sb02mt_(&JOBG,&JOBL,&FACT,&UPLO,&N,&M,A_copy.data(),&LDA,B_copy.data(),&LDB,Q_copy.data(),&LDQ,R_copy.data(),&LDR,L.data(),&LDL,IPIV.data(),&OUFACT,G.data(),&LDG,IWORK.data(),DWORK.data(),&LDWORK,&INFO);
    Eigen::Matrix<double,3,6> K;
    if (INFO!=0){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "sb02mt_ returned INFO=%d", INFO);
        // Consider throwing or returning a default result. We'll return zeroed K and G for now.
        Eigen::Matrix<double,3,6> K_zero = Eigen::Matrix<double,3,6>::Zero();
        return LQRsolveResult(K_zero, G,INFO);
        
    }  
    char DICO='D',HINV='D',SCAL='N',SORT='U';
    std::vector<double> WR(2*N,0),WI(2*N,0),RCOND(2*N,0);
    int BWORK[8*N];
    Eigen::Matrix<double,12,12> S=Eigen::Matrix<double,12,12>::Zero();
    Eigen::Matrix<double,12,6>U=Eigen::Matrix<double,12,6>::Zero();
    int LDS=2*N,LDU=2*N,INFO1=0;
    //A_copy=A;Q_copy=Q; R_copy=R;
    {
      double detA = A_copy.determinant();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calling sb02md_: det(A)=%.6g, A(0,0)=%.6g, G(0,0)=%.6g", detA, A_copy(0,0), G(0,0));
      Eigen::EigenSolver<Eigen::Matrix<double,6,6>> es(A_copy);
      for (int i=0;i<6;++i){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "eigA[%d]=% .6g%+.6gi", i, es.eigenvalues()[i].real(), es.eigenvalues()[i].imag());
      }
    }
    sb02md_(&DICO,&HINV,&UPLO,&SCAL,&SORT,&N,A_copy.data(),&LDA,G.data(),&LDG,Q_copy.data(),&LDQ,RCOND.data(),WR.data(),WI.data(),S.data(),&LDS,U.data(),&LDU,IWORK.data(),DWORK.data(),&LDWORK,BWORK,&INFO1);
    if (INFO1!=0){
        //Some Error handling here. Also check that BRB in invertible
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "sb02md_ returned INFO=%d", INFO1);
        // Consider throwing or returning a default result. We'll return zeroed K and G for now.
        Eigen::Matrix<double,3,6> K_zero = Eigen::Matrix<double,3,6>::Zero();
        return LQRsolveResult(K_zero, G,INFO1);
    }
    Eigen::Matrix<double,6,6>U11=U.topRows(6);
    Eigen::Matrix<double,6,6>U21=U.bottomRows(6);
    Eigen::MatrixXd X=U21*U11.inverse();
    K=R.inverse()*B.transpose()*X;
    
    return LQRsolveResult(K,G,INFO1);    

}
    */




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