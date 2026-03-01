
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
import scipy.linalg

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
        print("[ERROR], yaml file not found")
        
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
    ocp.solver_options.sim_method_num_steps=4

    nx = acados_model.x.size()[0]
    nu = acados_model.u.size()[0]

    # ----------------------------------
    # Cost: LINEAR_LS (yref-based)
    # ----------------------------------
    # ----------------------------------
    ocp.cost.cost_type   = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # States you care about: u=0, q=4, r=5
    idx_states   = [0, 1, 2,3,4]
    idx_controls = [0, 1, 2]

    n_y  = len(idx_states) + len(idx_controls)  # 8
    n_ye = len(idx_states)                       # 5

    # Vx: (8, nx) — selects only u, q, r from state vector
    Vx = np.zeros((n_y, nx))
    for i, idx in enumerate(idx_states):
        Vx[i, idx] = 1.0

    # Vu: (8, nu) — selects all 3 controls, placed in lower block
    Vu = np.zeros((n_y, nu))
    for i, idx in enumerate(idx_controls):
        Vu[len(idx_states) + i, idx] = 1.0

    # W: (8, 8) — only track what you care about, well conditioned
    Q_tracked = np.diag([
    Q[0, 0],   # u
    Q[1, 1],   # q
    Q[2, 2],   # r
    Q[3, 3],   # theta
    Q[4, 4],   # psi
])
    R_tracked = np.diag([R[0,0], R[1,1], R[2,2]])  # weights for controls

    W = scipy.linalg.block_diag(Q_tracked, R_tracked)

    ocp.cost.Vx = Vx  # (8, nx)
    ocp.cost.Vu = Vu  # (8, nu)
    ocp.cost.W  = W   # (8, 8)

    # Terminal cost — same state selection, no controls
    Vx_e = np.zeros((n_ye, nx))
    for i, idx in enumerate(idx_states):
        Vx_e[i, idx] = 1.0

    Q_e_tracked = np.diag([Qe[0,0], Qe[1,1], Qe[2,2], Qe[3,3],Qe[4,4]])

    ocp.cost.Vx_e = Vx_e       # (5, nx)
    ocp.cost.W_e  = Q_e_tracked # (5, 5)

    # References must match ny=6 and ny_e=3
    ocp.cost.yref   = np.zeros(n_y)   # [u, q, r, theta, psi, u1, u2, u3]
    ocp.cost.yref_e = np.zeros(n_ye)  # [u, q, r, theta, psi]

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
    ocp.constraints.idxbx = np.array([7]) 
    ocp.constraints.lbx = np.array([-1.4])
    ocp.constraints.ubx = np.array([1.4])  
    ocp.dims.nbx = 1 

    # ----------------------------------
    # Initial state constraint (must be updated before solve)
    # ----------------------------------
    ocp.constraints.x0 = np.zeros(nx)

    # ----------------------------------
    # Solver options
    # ----------------------------------
    print("W shape:", ocp.cost.W.shape)
    print("W diagonal:", np.diag(ocp.cost.W))
    print("W_e shape:", ocp.cost.W_e.shape)  
    print("W_e diagonal:", np.diag(ocp.cost.W_e))
    print("Vx shape:", ocp.cost.Vx.shape)
    print("Vx_e shape:", ocp.cost.Vx_e.shape)
    print("yref:", ocp.cost.yref)
    print("yref_e:", ocp.cost.yref_e)
    print("ny:", ocp.dims.ny)
    print("ny_e:", ocp.dims.ny_e)
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_warm_start=1
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    #ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"   # fast real-time iteration
    #ocp.solver_options.nlp_solver_max_iter = 100

    #ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    ocp.solver_options.levenberg_marquardt = 1e-4
    ocp.solver_options.print_level = 2

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
