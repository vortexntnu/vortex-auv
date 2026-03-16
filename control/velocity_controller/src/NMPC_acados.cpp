//#include "rclcpp/rclcpp.hpp"
#include "velocity_controller/NMPC_acados.hpp"
#include <cstring>   // memcpy
#include <cmath>
//#include "acados_solver_auv_model.h"
#include <iostream>

void AuvNMPC::set_diag(double* M, int n, const std::vector<double>& diag) {
    for (int i = 0; i < n; ++i) {
        std::memset(M + i*n, 0, n * sizeof(double));
        if (i < (int)diag.size()) M[i*n + i] = diag[i];
    }
}

AuvNMPC::~AuvNMPC() {
    if (capsule_) {
        auv_model_acados_free(capsule_);
        auv_model_acados_free_capsule(capsule_);
        capsule_ = nullptr;
        std::cout << "Destroying AuvNMPC..." << std::endl;
    }
}

bool AuvNMPC::init() {
    capsule_ = auv_model_acados_create_capsule();
    if (!capsule_) return false;

    
    int status = auv_model_acados_create(capsule_);
    if (status) {
        std::cerr << "[AuvNMPC] create failed, status: " << status << std::endl;
        return false;
    }

    solver_ = auv_model_acados_get_nlp_solver(capsule_);
    config_ = auv_model_acados_get_nlp_config(capsule_);
    dims_   = auv_model_acados_get_nlp_dims(capsule_);
    nlp_in_ = auv_model_acados_get_nlp_in(capsule_);
    nlp_out_= auv_model_acados_get_nlp_out(capsule_);
    N_ = (N_override_ > 0) ? N_override_ : AUV_MODEL_N; // fallback
    return true;
}
bool AuvNMPC::initialize_guess(std::array<double,NX> x,std::array<double,NU> u_init){
    for (int i=0;i<N_;++i){
        ocp_nlp_out_set(config_,dims_,nlp_out_,nlp_in_,i,"x",x.data());
        ocp_nlp_out_set(config_,dims_,nlp_out_,nlp_in_,i,"u",u_init.data());
    }
    return true;

    
}

void AuvNMPC::set_weights(const std::vector<double>& Wd, const std::vector<double>& We) {
    std::vector<double> W_diag_(NY, 0.0);
    std::vector<double> W_e_diag_(NY_E, 0.0);
    if ((int)Wd.size() == NY) {
        W_diag_ = Wd;
    } else {
        std::cerr << "[AuvNMPC] set_weights: W size mismatch, got " 
                  << Wd.size() << " expected " << NY << std::endl;
    }
    if ((int)We.size() == NY_E) {
        W_e_diag_ = We;
    } else {
        std::cerr << "[AuvNMPC] set_weights: W_e size mismatch, got "
                  << We.size() << " expected " << NY_E << std::endl;
    }
    // Build W and W_e from current diagonals (could cache)
    
    set_diag(W_.data(), NY, W_diag_);
    
    set_diag(W_e_.data(), NY_E, W_e_diag_);
}

void AuvNMPC::set_max_force(double max_force) {
    max_force2_ = max_force;
    if (std::isnan(max_force2_)) std::cout<<"Max force is Nan"<<std::endl;
    std::cout<<"max_force2:"<<max_force2_<<std::endl;
}

int AuvNMPC::solve_once()
{
    // Pin x0 at stage 0: idxbx = 0..NX-1; lbx=ubx=x0
    int idxbx0[NX]; for (int i=0;i<NX;++i) idxbx0[i]=i;
    ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, 0, "lbx",   const_cast<double*>(x0.data()));
    ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, 0, "ubx",   const_cast<double*>(x0.data()));

    

    // Update stages
    // Stage yref: [u, q, r, theta, psi, tau1, tau2, tau3]
    // Matches Vx selecting states [0,4,5,7,8] and Vu selecting all 3 inputs
    double yref[NY] = {
        xr[0],  // u     (surge velocity)
        xr[1],  // q     (pitch rate)
        xr[2],  // r     (yaw rate)
        xr[3],  // theta (pitch)
        xr[4],  // psi   (yaw)
        ur[0],  // tau_surge
        ur[1],  // tau_pitch
        ur[2]   // tau_yaw
    };
    for (int k = 0; k < N_; ++k) {
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, k, "yref", yref);
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, k, "W",    W_.data());
    }
    // Terminal yref_e: [u, q, r, theta, psi]
    double yref_e[NY_E] = {
        xr[0],  // u
        xr[1],  // q
        xr[2],  // r
        xr[3],  // theta
        xr[4]   // psi
    };

    ocp_nlp_cost_model_set(config_, dims_, nlp_in_, N_, "yref_e", yref_e);
    ocp_nlp_cost_model_set(config_, dims_, nlp_in_, N_, "W_e",    W_e_.data());

    // Solve (blocking)
    /*
    for (int k = 0; k <= N_; ++k) {
        ocp_nlp_out_set(config_, dims_, nlp_out_, nlp_in_,k, "x", const_cast<double*>(x0.data()));
    }
    double u_init[NU] = {0.0, 0.0, 0.0};
    for (int k = 0; k < N_; ++k) {
        ocp_nlp_out_set(config_, dims_, nlp_out_,nlp_in_, k, "u", u_init);
    }*/
    
    int status = auv_model_acados_solve(capsule_);
    
    if (status == 0) {
    std::cout << "--- Predicted Trajectory ---" << std::endl;
    for (int k = 0; k <= N_; ++k) {
        double x_k[NX];
        ocp_nlp_out_get(config_, dims_, nlp_out_, k, "x", x_k);
        std::cout << "Step " << k << ": " <<x_k[0] <<"m/s, "<<x_k[4] <<"rad/s, "<<x_k[5] <<"rad/s, "<< x_k[7] << " rad, " <<x_k[8]<<"rad"<< std::endl;
    }
    double temp[NY];
    ocp_nlp_cost_model_get(config_,dims_,nlp_in_,0,"yref",temp);
    std::cout<< "Reference: "<<temp[0]<<", "<<temp[1]<<", "<<temp[2]<<", "<<temp[3]<<","<<temp[4]<<std::endl;
} else {
    std::cout << "[AuvNMPC] Solver failed with status: " << status << std::endl;
}

    // Read u0
    double u0[NU] = {0};
    ocp_nlp_out_get(config_, dims_, nlp_out_, 0, "u", u0);
    for (int i=0;i<NU;++i) u0_out[i] = u0[i];

    return status;
}

std::vector<double> AuvNMPC::getU0(){
    return u0_out;
} 

void AuvNMPC::setState(const std::array<double, 9>& x){
    x0=x;
    for (int i=0;i<x.size();i++){
        if (std::isnan(x0[i])){
            std::cout << "x0[" << i << "] is NaN!" << std::endl;
        }
        std::cout<<x0[i]<<",";
    }
    std::cout<<std::endl;

};

void AuvNMPC::setReference(const std::array<double, NY-NU>& x_ref){ //surge, pitch_rate, yaw_rate, pitch, yaw
    xr=x_ref; 
    std::cout<<"xr: ";
    for (int i=0;i<NY-NU;i++){
        if (std::isnan(xr[i])){
            std::cout << "xr[" << i << "] is NaN!" << std::endl;
        }
        std::cout<<xr[i]<<",";
    }
    std::cout<<std::endl;

    
};

