#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <casadi/core/generic_type.hpp>
#include <casadi/core/sparsity_interface.hpp>
#include <casadi/core/sx_fwd.hpp>
#include <cmath>
#include <rclcpp/logger.hpp>
#include "rclcpp/rclcpp.hpp"
#include "velocity_controller/utilities.hpp"
#include "velocity_controller/NMPC_setup.hpp"


bool NMPC_controller::set_matrices(std::vector<double> Q,std::vector<double> R,std::vector<double> inertia_matrix, double max_force,std::vector<double> water_r_low,std::vector<double> water_r_high){
    if (Q.size()!=9){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The Q matrix has the wrong amount of elements");
        return 0;
    }
    if(R.size()!=3){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The R matrix has the wrong amount of elements");
        return 0;
    }
    if(inertia_matrix.size()!=36){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The M matrix has the wrong amount of elements");
        return 0;
    }
    if(water_r_low.size()!=36||water_r_high.size()!=36){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The D matrix has the wrong amount of elements");
        return 0;
    }
    if (max_force<0){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"The max_force need to be >0");
        return 0;
    }
    if (inertia_matrix[0]<0){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Negative mass?");
        return 0;
    }
    Q_.setZero();
    R_.setZero();
    Q_.diagonal() = Eigen::Map<Eigen::VectorXd>(Q.data(), Q.size());
    R_.diagonal() = Eigen::Map<Eigen::VectorXd>(R.data(), R.size());
    Eigen::Matrix<double,6,6> inertia_matrix_ = Eigen::Map<const Eigen::Matrix<double,6,6>>(inertia_matrix.data(),6,6);
    D_low=Eigen::Map<const Eigen::Matrix<double,6,6>>(water_r_low.data(),6,6);
    D_high=Eigen::Map<const Eigen::Matrix<double,6,6>>(water_r_high.data(),6,6);
    M_inv=inertia_matrix_.inverse();
    mass=inertia_matrix[0];
    Ix=inertia_matrix_(3,3);
    Iy=inertia_matrix_(4,4);
    Iz=inertia_matrix_(5,5);

    //B_=M_inv*(Eigen::Matrix<double,6,3>()  << 1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0).finished();

    return true;
};
void NMPC_controller::reset_controller(){
    return;
}
bool NMPC_controller::set_interval(double interval){
    interval_=interval;
    return true;
}
bool NMPC_controller::initialize_MPC(){
    using SYM=casadi::MX;
    SYM X=SYM::sym("X",n,1); //u,v,w,p,q,r,phi,theta,psi
    SYM A=SYM::zeros(n,n);
    casadi::DM M_i=casadi::DM::zeros(6,6);
    SYM U=SYM::sym("U",3,1);
    casadi::DM Q=casadi::DM::zeros(n,n);
    casadi::DM R=casadi::DM::zeros(m,m);
    /*U(0,0)=SYM::sym("u_surge");
    U(1,0)=SYM::sym("u_pitch");
    U(2,0)=SYM::sym("u_yaw");*/

    //Creating M_i matrix
    for (int i=0;i<M_inv.rows();i++){
        for (int j=0;j<M_inv.cols();j++){
            M_i(i,j)=M_inv(i,j);
        }
    }
    //Creating Q matrix
    for (int i=0;i<Q_.rows();i++){
        for (int j=0;j<Q_.cols();j++){
            Q(i,j)=Q_(i,j);
        }
    }

    //creating R matrix
    for (int i=0;i<R_.rows();i++){
        for (int j=0;j<R_.cols();j++){
            R(i,j)=R_(i,j);
        }
    }
    //creating D matrix for now only linear dampening
    SYM D=SYM::zeros(6,6); //TODO: optimized for linear matrix?
    for (int i=0;i<D_low.rows();i++){
        for (int j=0;j<D_low.cols();j++){
            D(i,j)=D_low(i,j);
        }
    }
    D=mtimes(M_i,D);
    //Creating Coriolis matrix
    SYM Cor=SYM::zeros(6,6); //TODO maybe make a general crossproduct matric generator
    Cor(0,1)=-mass*X(2); Cor(0,2)=-mass*X(1);
    Cor(1,0)=mass*X(2); Cor(1,2)=-mass*X(0);
    Cor(2,0)=-mass*X(1); Cor(2,1)=mass*X(0);

    Cor(3,4)=-Iz*X(5); Cor(3,5)=-Iy*X(4);
    Cor(4,3)=Iz*X(5); Cor(4,5)=-Ix*X(3);
    Cor(5,3)=-Iy*X(4); Cor(5,4)=Ix*X(3);
    Cor=mtimes(M_i,Cor);
    //Creating transformation matrix, body rotation to angles
    SYM T=SYM::zeros(3,3);
    SYM eps = SYM::sym("eps");
    eps=1e-6;
    T(0,0)=1; T(0,1)=sin(X(6))*sin(X(7))/fmax(cos(X(7)),eps); T(0,2)=cos(X(6))*sin(X(7))/fmax(cos(X(7)),eps);
    T(1,1)=cos(X(6)); T(1,2)=-sin(X(6));
    T(2,1)=sin(X(6))/fmax(cos(X(7)),eps); T(2,2)=cos(X(6))/fmax(cos(X(7)),eps);

    //Creating A matrix
    for (int i=0;i<6;i++){
        for (int j=0;j<6;j++){
            A(i,j)=Cor(i,j)+D(i,j);
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            A(6+i,3+j)=T(i,j);
        }
    }
    
    SYM temp=SYM::zeros(9,1);
    temp(0)=U(0); //Force surge
    temp(4)=U(1); //Moment pitch
    temp(5)=U(2); //Moment yaw
    SYM temp2=SYM::zeros(9,9);
    for (int i=0;i<6;i++){
        for (int j=0;j<6;j++){
            temp2(i,j)=M_inv(i,j);
        }
    }
    
    
    casadi::MXDict ode;
    ode["x"]=X;
    ode["p"]=U;
    SYM rhs=SYM::zeros(9,1);
    //rhs=A*X+temp2*temp; //    
    rhs=-mtimes(A,X)+mtimes(temp2, temp); 

    ode["ode"]=rhs;
    casadi::Dict opts;
    opts["tf"]=interval_;
    /*
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"X: %d x %d", X.size1(), X.size2());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"U: %d x %d", U.size1(), U.size2());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"A: %d x %d", A.size1(), A.size2());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"temp2: %d x %d", temp2.size1(), temp2.size2());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"temp: %d x %d", temp.size1(), temp.size2());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"rhs: %d x %d", rhs.size1(), rhs.size2());
    */
    casadi::Function Fint=casadi::integrator("Fint","cvodes",ode,opts);
    
    
    //Decision variables
    std::vector<SYM> X_v(N+1), U_v(N);
    for (int i=0;i<=N;i++) X_v[i] = SYM::sym("X_"+std::to_string(i),n);
    for (int i=0;i<N;i++) U_v[i] = SYM::sym("U_"+std::to_string(i),m);
    //Stacked the decision variables
    std::vector<SYM> z_parts;
    z_parts.insert(z_parts.end(), X_v.begin(), X_v.end());
    z_parts.insert(z_parts.end(), U_v.begin(), U_v.end());
    SYM Z = vertcat(z_parts);
    
    //Initial state
    SYM x0=SYM::sym("x0",n);
    SYM xr=SYM::sym("xr",n);
    SYM ur=SYM::sym("ur",m);
    
    SYM P=SYM::vertcat({x0,xr,ur});


    auto p_x0 = P(casadi::Slice(0, n));                       // x0
    auto p_xr = P(casadi::Slice(n, 2*n));                     // xr
    auto p_ur = P(casadi::Slice(2*n, 2*n + m));               // ur

    //Dynamic constraints

    std::vector<SYM> g_list;
    g_list.push_back(X_v[0]-p_x0);
    for (int i=0; i<N;i++){
        casadi::MXDict integrator_in;
        integrator_in["x0"] = X_v[i];
        integrator_in["p"] = U_v[i];
        SYM X_next = Fint(integrator_in).at("xf");
        g_list.push_back(X_v[i+1]-X_next);
    }
    SYM G=vertcat(g_list);

    //Wheights
    SYM J=SYM::zeros(1);

    for (int k = 0; k < N; ++k) {
        SYM ex = X_v[k] - p_xr;
        SYM eu = U_v[k] - p_ur;
        J += mtimes(mtimes(ex.T(), Q), ex) + mtimes(mtimes(eu.T(), R), eu);
    }
    {
        casadi::DM QN=Q;
        SYM eN = X_v[N] - p_xr;
        J += mtimes(mtimes(eN.T(), QN), eN);
    }
    //Create bounds

    casadi::DM x_min = -casadi::DM::inf(n,1);
    casadi::DM x_max =  casadi::DM::inf(n,1);
    casadi::DM u_min = -100.0*casadi::DM::ones(m,1);
    casadi::DM u_max =  100.0*casadi::DM::ones(m,1);

    std::vector<casadi::DM> lbx_parts, ubx_parts;
    // X0..XN
    for (int k = 0; k <= N; ++k) {
        lbx_parts.push_back(x_min);
        ubx_parts.push_back(x_max);
    }
    // U0..U_{N-1}
    for (int k = 0; k < N; ++k) {
        lbx_parts.push_back(u_min);
        ubx_parts.push_back(u_max);
    }
    lbx = vertcat(lbx_parts);
    ubx = vertcat(ubx_parts);

    // Equality constraints: G == 0
    lbg = casadi::DM::zeros(n*(N+1));
    ubg = casadi::DM::zeros(n*(N+1));


    //building NLP
    casadi::MXDict nlp;
    nlp["x"]=Z;
    nlp["f"]=J;
    nlp["g"]=G;
    nlp["p"]=P;
    casadi::Dict opts1;
    opts1["ipopt.print_level"]=0;
    opts1["print_time"]=false;
    opts1["ipopt.sb"]="yes";

    solver=casadi::nlpsol("solver","ipopt",nlp,opts1);
    
    // --------------------------------------------------------
    // Prepare parameter vector Pval and initial guess Z0
    // --------------------------------------------------------
    // Example numeric values:
    casadi::DM x0_val = casadi::DM::zeros(n,1);
    std::vector<casadi::DM> Pval_parts;
    Pval_parts.push_back(x0_val);
    Pval_parts.push_back(casadi::DM::zeros(n,1));  
    Pval_parts.push_back(casadi::DM::zeros(m,1));  
    Pval = vertcat(Pval_parts);

    // Initial guess: Z0 (X guesses then U guesses)
    std::vector<casadi::DM> Z0_parts;
    for (int k = 0; k <= N; ++k) Z0_parts.push_back(x0_val);      // start with x0 everywhere
    for (int k = 0; k <  N; ++k) Z0_parts.push_back(casadi::DM::zeros(m,1));
    Z0_next = vertcat(Z0_parts);


    return true;
}

Eigen::Matrix<double, 3, 1> NMPC_controller::calculate_thrust(Guidance_data guidance_values, State state){
    
    casadi::DM x0_val={state.surge,state.sway,state.heave,state.roll_rate,state.pitch_rate,state.yaw_rate,state.roll,state.pitch,state.yaw};
    casadi::DM xr_val={guidance_values.surge,guidance_values.sway,guidance_values.heave,guidance_values.roll_rate,guidance_values.pitch_rate,guidance_values.yaw_rate,guidance_values.roll,guidance_values.pitch,guidance_values.yaw};
    casadi::DM ur_val=casadi::DM::zeros(m);
    Pval=casadi::DM::vertcat({x0_val,xr_val,ur_val});
  // Solve
 
    casadi::DMDict solver_in;
    solver_in["x0"] = Z0_next;
    solver_in["lbx"] = lbx;
    solver_in["ubx"] = ubx;
    solver_in["lbg"] = lbg;
    solver_in["ubg"] = ubg;
    solver_in["p"] = Pval;
    auto sol = solver(solver_in);
    if (sol.count("x") == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "NLP solver failed");
        return {9999,9999,9999}; //TODO: check for 9999,9999,9999
        }
    casadi::DM Zstar = sol.at("x");  // optimal stacked decision vector
    //TODO: check to see NAN or INF values in solution
    
  // index of U0 start:
  int offset_u0 = n*(N+1);
  // Extract u0* (first control block after all states)
  // Z = [X0; X1; ...; XN; U0; U1; ...; U_{N-1}]
  casadi::DM u0_star = Zstar(casadi::Slice(offset_u0, offset_u0 + m));

  std::cout << "u0* = " << u0_star << std::endl;


  // Warm-start shift for next iteration
  // Build new Z0_next = [X1*; X2*; ...; XN*; XN*; U1*; ...; U_{N-1}*; U_{N-1}*]
  // (X0 will be re-anchored by the new measured x0 in constraints)
  std::vector<casadi::DM> Xstar(N+1), Ustar(N);
  // Unstack from Zstar:
  for (int k = 0; k <= N; ++k) {
    int i0 = k*n;
    Xstar[k] = Zstar(casadi::Slice(i0, i0+n));
  }
  for (int k = 0; k < N; ++k) {
    int i0 = n*(N+1) + k*m;
    Ustar[k] = Zstar(casadi::Slice(i0, i0+m));
  }

  std::vector<casadi::DM> Z0_next_parts;
  // shifted states: X1..XN, repeat XN at end
  for (int k = 1; k <= N; ++k) Z0_next_parts.push_back(Xstar[k]);
  Z0_next_parts.push_back(Xstar[N]);
  // shifted inputs: U1..U_{N-1}, repeat last
  for (int k = 1; k <  N; ++k) Z0_next_parts.push_back(Ustar[k]);
  Z0_next_parts.push_back(Ustar[N-1]);

  Z0_next = vertcat(Z0_next_parts);
  
    Eigen::Matrix<double, 3, 1> result;
    result(0) = double(u0_star(0));
    result(1) = double(u0_star(1));
    result(2) = double(u0_star(2));
    return result;
}
