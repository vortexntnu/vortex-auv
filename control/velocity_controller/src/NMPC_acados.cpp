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

    // Get N from generated getter if available; else use override.
    // If your header doesn’t have *_get_N, pass N in the constructor.
    #ifdef auv_model_acados_get_N
    N_ = auv_model_acados_get_N(capsule_);
    #else
    N_ = (N_override_ > 0) ? N_override_ : 20; // fallback
    #endif

    // Provide some safe default weights, tune later or call set_weights()
    if (W_diag_.size() == NY) {
        // Example diag: [Q; R]
        // states:  5,5,8, 1,1,1, 10,15,10
        // inputs:  1,0.5,0.5
        double Wd[NY] = {5,5,8, 1,1,1, 10,15,10, 1,0.5,0.5};
        W_diag_.assign(Wd, Wd + NY);
    }
    if (W_e_diag_.size() == NY_E) {
        double Wed[NY_E] = {10,10,15, 2,2,2, 30,40,30};
        W_e_diag_.assign(Wed, Wed + NY_E);
    }

    // Initialize per-stage yref & W (zeros ref by default)
    std::vector<double> W(NY * NY, 0.0);
    set_diag(W.data(), NY, W_diag_);
    for (int k = 0; k < N_; ++k) {
        double yref[NY] = {0};
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, k, "yref", yref);
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, k, "W",    W.data());
    }
    {
        std::vector<double> W_e(NY_E * NY_E, 0.0);
        set_diag(W_e.data(), NY_E, W_e_diag_);
        double yref_e[NY_E] = {0};
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, N_, "yref", yref_e);
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, N_, "W",    W_e.data());
    }

    // Initialize nonlinear constraint bounds at stage 0 (if you defined con_h_expr with nh=1)
    double lh0[1] = { 0.0 };
    double uh0[1] = { max_force2_ };
    ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, 0, "lh", lh0);
    ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, 0, "uh", uh0);

    return true;
}

void AuvNMPC::set_weights(const std::vector<double>& Wd, const std::vector<double>& We) {
    for (int i=0;i<(int)Wd.size();i++){
        std::cout<<Wd[i]<<",";
    }
    std::cout<<std::endl;
    if ((int)Wd.size() == NY){
        std::cout<<"initializing Wd"<<std::endl;
        W_diag_ = Wd;
    }
    if ((int)We.size() == NY_E){ 
        std::cout<<"Initializing We"<<std::endl;
        W_e_diag_ = We;}
    for (int i=0;i<NY;i++){
        std::cout<<"Weights "<<i<<":"<<W_diag_[i]<<",";
    }
    std::cout<<std::endl;
    for (int i=0;i<NY_E;i++){
        std::cout<<"Weights "<<i<<":"<<W_e_diag_[i]<<",";
    }
    std::cout<<std::endl;
}

void AuvNMPC::set_max_force(double max_force) {
    max_force2_ = max_force;
    if (std::isnan(max_force2_)) std::cout<<"Max force is Nan"<<std::endl;
    std::cout<<"max_force2:"<<max_force2_<<std::endl;
}

int AuvNMPC::solve_once()
{
    // Pin x0 at stage 0: idxbx = 0..NX-1; lbx=ubx=x0
    std::cout<<"N:"<<N_<<std::endl;
    //int idxbx0[NX]; for (int i=0;i<NX;++i) idxbx0[i]=i;
    //ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, 0, "lbx",   const_cast<double*>(x0.data()));
    ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, 0, "ubx",   const_cast<double*>(x0.data()));

    // Build W and W_e from current diagonals (could cache)
    std::vector<double> W(NY * NY, 0.0);
    set_diag(W.data(), NY, W_diag_);
    std::vector<double> W_e(NY_E * NY_E, 0.0);
    set_diag(W_e.data(), NY_E, W_e_diag_);

    // Update stages
    for (int k = 0; k < N_; ++k) {
        double yref[NY] = {0};
        std::memcpy(yref,      xr.data(), NX*sizeof(double));
        std::memcpy(yref+NX,   ur.data(), NU*sizeof(double));
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, k, "yref", yref);
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, k, "W",    W.data());

        double lh[1] = { 0.0 };
        double uh[1] = { max_force2_ };
        ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, k, "lh", lh);
        ocp_nlp_constraints_model_set(config_, dims_, nlp_in_, nlp_out_, k, "uh", uh);
    }
    

    {
        double yref_e[NY_E] = {0};
        std::memcpy(yref_e, xr.data(), NX*sizeof(double));
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, N_, "yref", yref_e);
        ocp_nlp_cost_model_set(config_, dims_, nlp_in_, N_, "W",    W_e.data());
    }

    // Solve (blocking)
    int status = auv_model_acados_solve(capsule_);

    // Read u0
    double u0[NU] = {0};
    ocp_nlp_out_get(config_, dims_, nlp_out_, 0, "u", u0);
    for (int i=0;i<NU;++i) u0_out[i] = u0[i];

    return status;
}

std::vector<double> AuvNMPC::getU0(){
    return u0_out;
} 

void AuvNMPC::setState(const std::array<double, NX>& x){
    x0=x;
    for (int i=0;i<NX;i++){
        if (std::isnan(x0[i])){
            std::cout << "x0[" << i << "] is NaN!" << std::endl;
        }
        std::cout<<x0[i]<<",";
    }
    std::cout<<std::endl;

};

void AuvNMPC::setReference(const std::array<double, NX>& x_ref, const std::array<double, NU>& u_ref){
    xr=x_ref;
    ur=u_ref;
    std::cout<<"xr: ";
    for (int i=0;i<NX;i++){
        if (std::isnan(xr[i])){
            std::cout << "xr[" << i << "] is NaN!" << std::endl;
        }
        std::cout<<xr[i]<<",";
    }
    std::cout<<"ur: ";
    for (int i=0;i<NU;i++){
        if (std::isnan(ur[i])){
            std::cout << "ur[" << i << "] is NaN!" << std::endl;
        }
        std::cout<<ur[i]<<",";
    }
    std::cout<<std::endl;

    
};