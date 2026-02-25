
#!/usr/bin/env python3
"""
AUV NMPC OCP generator for acados
N = 20, Tf = 0.05 (2.5 ms steps)
Nonlinear thrust magnitude constraint.
Diagonal Q/R/Qe loaded from weights.yaml or defaults.

Generates a solver in ./build_auv_solver/
"""

import numpy as np
import yaml
from pathlib import Path

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat

# Import the underwater vehicle model from Step 1
from dynamics import make_auv_model


# -----------------------------
# Load weighting matrices
# -----------------------------
def load_matrices(path="../config/parameters.yaml"):
    if Path(path).exists():
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        node_key = next(iter(data.keys()))
        print("Top-level keys:", list(data.keys()))
        print("[INFO] Loading weights from", path)
        Q = np.diag(data[node_key]["ros__parameters"]["NMPC_params"]["Q"])
        R = np.diag(data[node_key]["ros__parameters"]["NMPC_params"]["R"])
        Qe = np.diag(data[node_key]["ros__parameters"]["NMPC_params"]["Q"])
        inertia_M=np.reshape(data[node_key]["ros__parameters"]["inertia_matrix"],(6,6))
        D_lin = np.reshape(data[node_key]["ros__parameters"]["dampening_matrix_low"],(6,6))
        D_quad = np.reshape(data[node_key]["ros__parameters"]["dampening_matrix_high"],(6,6))
        N=data[node_key]["ros__parameters"]["NMPC_params"]["N"]
        delta_t=data[node_key]["ros__parameters"]["publish_rate"]
        max_force=data[node_key]["ros__parameters"]["max_force"]
    else:
        print("[INFO] Using default Q/R/Qe weights.")
        # Default weights — to be removed
        Q  = np.diag([ 5, 5, 8, 1, 1, 1, 10, 15, 10 ])
        R  = np.diag([ 1.0, 0.5, 0.5 ])
        Qe = np.diag([10,10,15, 2,2,2, 30,40,30 ])
    return Q, R, Qe, inertia_M, D_lin, D_quad,N,delta_t,max_force


# -----------------------------
# Build the OCP
# -----------------------------
def create_auv_ocp():
    # Load weights
    Q, R, Qe, inertia_Matrix, D_lin, D_quad, N, delta_t, max_force = load_matrices()

    # Load dynamical model
    mdl = make_auv_model(inertia_Matrix,D_lin,D_quad)

    # Wrap into acados model format
    acados_model = AcadosModel()
    acados_model.name = mdl["name"]
    acados_model.x = mdl["x"]
    acados_model.u = mdl["u"]
    acados_model.f_expl_expr = mdl["f_expl_expr"]
    #acados_model.f_impl_expr = mdl["f_impl_expr"]
    
    # Create OCP
    ocp = AcadosOcp()
    ocp.model = acados_model

    # Horizon settings
    Tf = (delta_t*N)/1000    # total horizon [seconds]
    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    ocp.solver_options.integrator_type="ERK"
    ocp.solver_options.sim_method_num_stages=4
    ocp.solver_options.sim_method_num_steps=2

    nx = acados_model.x.size()[0]
    nu = acados_model.u.size()[0]

    # ----------------------------------
    # Cost: LINEAR_LS (yref-based)
    # ----------------------------------
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # Select which states and inputs enter the cost

    Vx = np.zeros((nx + nu, nx))  # 12×9
    Vu = np.zeros((nx + nu, nu))  # 12×3

    # Top-left block (state tracking)
    Vx[0:nx, 0:nx] = np.eye(nx)

    # Bottom-right block (input tracking)
    Vu[nx:nx + nu, 0:nu] = np.eye(nu)

    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu

    ocp.cost.W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])

    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.W_e = Qe

    # Default references (0 until updated at runtime)
    ocp.cost.yref  = np.zeros(nx + nu)
    ocp.cost.yref_e = np.zeros(nx)

    # ----------------------------------
    # Nonlinear input constraint:
    # Fx^2 + My^2 + Mz^2 <= 10000
    # ----------------------------------
    #Fx, My, Mz = acados_model.u[0], acados_model.u[1], acados_model.u[2]
    #h_expr = Fx**2 + My**2 + Mz**2

    #ocp.model.con_h_expr = vertcat(h_expr)       # 1 constraint
    #ocp.dims.nh=1
    #ocp.constraints.lh = np.array([0.0]) # lower bound: h >= 0 (redundant)
    #ocp.constraints.uh = np.array([max_force**2])  # upper bound: magnitude <= 100

    # No bounds on slack variables (we don't use slacks)
    #ocp.constraints.idxsh = np.array([], dtype=int)
    #ocp.constraints.lsh = np.array([])
    #ocp.constraints.ush = np.array([])

    # No box-constraints on h (handled via lh/uh)
    #ocp.constraints.idxbh = np.array([], dtype=int)
    #ocp.constraints.lbh = np.array([])
    #ocp.constraints.ubh = np.array([])

    u_max = max_force     # from YAML

    ocp.constraints.lbu = -u_max * np.ones(nu)
    ocp.constraints.ubu =  u_max * np.ones(nu)
    ocp.constraints.idxbu = np.arange(nu, dtype=int)

    # ----------------------------------
    # Initial state constraint (must be updated before solve)
    # ----------------------------------
    ocp.constraints.x0 = np.zeros(nx)

    # ----------------------------------
    # Solver options
    # ----------------------------------
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_warm_start=1
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    #ocp.solver_options.integrator_type = "ERK"
    #cp.solver_options.nlp_solver_type = "SQP"   # fast real-time iteration
    ocp.solver_options.nlp_solver_max_iter = 100

    ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    ocp.solver_options.levenberg_marquardt = 1e-4
    ocp.solver_options.print_level = 2
    ocp.constraints.idxe = np.array([], dtype=int)

    # ----------------------------------
    # Output directory
    # ----------------------------------
    ocp.code_gen_opts.code_export_directory = "build_auv_solver"

    print("nh:", ocp.dims.nh)
    print("lh:", ocp.constraints.lh)
    print("uh:", ocp.constraints.uh)

    #print("idxbh:", ocp.constraints.idxbh)
    print("idxsh:", ocp.constraints.idxsh)

    return ocp


# -----------------------------
# Main entry: generate solver
# -----------------------------
if __name__ == "__main__":
    ocp = create_auv_ocp()
    print("[INFO] Generating AUV NMPC solver...")
    AcadosOcpSolver(ocp, json_file="auv_ocp.json")
    print("[INFO] Done. Solver code is in ./build_auv_solver/")
