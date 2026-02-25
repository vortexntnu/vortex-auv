#pragma once
//#include <vector>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include "velocity_controller/utilities.hpp"

class NMPC_controller{
    public:
    Eigen::Matrix<double, 3, 1> calculate_thrust(Guidance_data guidance_values, State state);
    bool set_matrices(std::vector<double> Q_,std::vector<double> R_,std::vector<double> inertia_matrix, double max_force,std::vector<double> water_r_low,std::vector<double> water_r_high);
    void reset_controller();
    bool set_interval(double interval);
    bool initialize_MPC();
    private:
    Eigen::Matrix<double,9,9> Q_;
    Eigen::Matrix<double,3,3>R_;
    Eigen::Matrix<double,6,6>M_inv;
    Eigen::Matrix<double,6,6>D_low;
    Eigen::Matrix<double,6,6>D_high;
    //Eigen::Matrix<double,6,3>B_;
    double interval_;
    double mass;
    double Iz;
    double Ix;
    double Iy;
    int N=3;
    int n=9;
    int m=3;
    casadi::DM Z0_next; //For warm start
    casadi::DM lbx;
    casadi::DM ubx;
    casadi::DM lbg;
    casadi::DM ubg;
    casadi::DM Pval;
    casadi::Function solver;


};