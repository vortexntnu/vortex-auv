
#pragma once
#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>

// acados C API (generated)
extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados/ocp_nlp/ocp_nlp_common.h"
#include "acados/utils/print.h"

// Generated solver headers
#include "acados_solver_auv_model.h"   // <-- from build_auv_solver/
}

class AuvNMPC {
public:
    // Adjust sizes if your model differs
    static constexpr int NX = 9;   // [u v w p q r phi theta psi]
    static constexpr int NU = 3;   // [Fx My Mz]
    static constexpr int NY = NX + NU;
    static constexpr int NY_E = NX;

    
// Pass N if your generated header does not provide <model>_acados_get_N()
    explicit AuvNMPC(int N_horizon_override = -1)
    : N_override_(N_horizon_override) {}

    ~AuvNMPC();


    // Not copyable
    AuvNMPC(const AuvNMPC&) = delete;
    AuvNMPC& operator=(const AuvNMPC&) = delete;

    // Lifecycle
    bool init();


    void set_weights(const std::vector<double>& W_diag, const std::vector<double>& W_e_diag);
    void set_max_force(double max_force); // updates bound on con_h: Fx^2+My^2+Mz^2 <= max^2

    // Inputs
    void setState(const std::array<double, NX>& x);
    void setReference(const std::array<double, NX>& x_ref, const std::array<double, NU>& u_ref);
    void setWeights(const std::vector<double>& W_diag, const std::vector<double>& W_e_diag); // sizes: NY, NY_E
    void setMaxForce(double max_force); // updates con_h bounds


    // One-shot solve: provide current state, desired state & input refs, get u0 back.
    // Returns solver status (0 == success).
    int solve_once();

    // Outputs
    std::vector<double> getU0(); 

    private:
    
    // generated capsule type (from acados_solver_auv_model.h)
    auv_model_solver_capsule* capsule_ = nullptr;


    // acados solver
    ocp_nlp_solver* solver_ = nullptr;
    ocp_nlp_config* config_ = nullptr;
    ocp_nlp_dims*   dims_   = nullptr;
    ocp_nlp_in*     nlp_in_ = nullptr;
    ocp_nlp_out*    nlp_out_= nullptr;

    int N_=20;
    int N_override_=-1;

    std::vector<double> W_diag_{NY,0.0};    // length NY
    std::vector<double> W_e_diag_{NY_E,0.0};  // length NY_E
    double max_force2_ = 100*100;   // squared constraint, default 100^2

    
    static void set_diag(double* M, int n, const std::vector<double>& diag);
    //U out
    std::vector<double> u0_out={0,0,0};
    //Recorded states states
    std::array<double,NX> x0;
    std::array<double,NX> xr;
    std::array<double,NU> ur={0,0,0};
};
