
# auv_model.py
from casadi import SX, vertcat, diag, cos, sin, tan, fabs, inv
import numpy as np
import yaml

def euler_kinematics_T(phi, theta):
    """
    Euler ZYX rate mapping: [phi_dot, theta_dot, psi_dot] = T(phi,theta) * [p q r]
    Avoid singularity at cos(theta)=0 with a tiny epsilon if needed (handled later in OCP).
    """
    T = SX.zeros(3, 3)
    T[0, 0] = 1.0
    T[0, 1] = sin(phi) * tan(theta)
    T[0, 2] = cos(phi) * tan(theta)
    T[1, 0] = 0.0
    T[1, 1] = cos(phi)
    T[1, 2] = -sin(phi)
    T[2, 0] = 0.0
    T[2, 1] = sin(phi) / cos(theta)
    T[2, 2] = cos(phi) / cos(theta)
    return T

def coriolis_rb_diag(m, Ix, Iy, Iz, u, v, w, p, q, r):
    """
    Rigid-body Coriolis/centripetal matrix C_RB for diagonal inertia matrices.
    Fossen 2011-style, for 6DOF body velocities [u v w p q r].
    """
    C = SX.zeros(6, 6)
    # Linear-Linear block (zero)
    # Linear-Angular block
    C[0, 4] =  m * w
    C[0, 5] = -m * v
    C[1, 3] = -m * w
    C[1, 5] =  m * u
    C[2, 3] =  m * v
    C[2, 4] = -m * u
    # Angular-Linear block
    C[3, 1] =  m * w
    C[3, 2] = -m * v
    C[4, 0] = -m * w
    C[4, 2] =  m * u
    C[5, 0] =  m * v
    C[5, 1] = -m * u
    # Angular-Angular block (J*omega x omega)
    C[3, 4] = -Iz * r
    C[3, 5] =  Iy * q
    C[4, 3] =  Iz * r
    C[4, 5] = -Ix * p
    C[5, 3] = -Iy * q
    C[5, 4] =  Ix * p
    return C

def dampening(linear_diag, quad_diag, u, v, w, p, q, r):
    """
    Linear + quadratic diagonal damping D(v)*v = (Dl + Dq*|v|) v.
    Inputs Dl, Dq are 6-vectors (or lists) for [u v w p q r].
    """
    Dl = SX(linear_diag)
    Dq_vec = SX(6, 1)
    vel = vertcat(u, v, w, p, q, r)
    abs_vel = SX(6, 1)
    for i in range(6):
        abs_vel[i] = fabs(vel[i])
    Dq = SX(quad_diag)  # elementwise times abs_vel when applied
    # Effective damping operator when multiplying right by vel:
    # (Dl + Dq*|vel|) * vel
    return Dl, Dq, abs_vel

def make_auv_model(mass_inertia_matrix,D_lin,D_quad):
    """
    Build symbolic CasADi model for a 9x3 AUV suitable for acados codegen.

    params (dict) expected keys:
      - m, Ix, Iy, Iz               : scalars
      - D_lin: length-6 list/tuple  : linear damping diag for [u v w p q r]
      - D_quad: length-6 list/tuple : quadratic damping diag
      - g_vec: length-6 list/tuple  : restoring forces/torques in body (optional; use zeros if unknown)
    Returns:
      dict with keys: x, u, f_expl_expr, name
    """
    # Unpack parameters
    m=SX(mass_inertia_matrix)
    Ix  = mass_inertia_matrix[3][3]
    Iy  = mass_inertia_matrix[4][4]
    Iz  = mass_inertia_matrix[5][5]
    #D_lin  = params.get('D_lin')
    #D_quad = params.get('D_quad')

    # States: body velocities and Euler angles
    u = SX.sym('u')  # surge
    v = SX.sym('v')  # sway
    w = SX.sym('w')  # heave
    p = SX.sym('p')  # roll rate NED
    q = SX.sym('q')  # pitch rate NED
    r = SX.sym('r')  # yaw rate NED
    phi   = SX.sym('phi') #Body
    theta = SX.sym('theta') #Body
    psi   = SX.sym('psi') #Body

    x = vertcat(u, v, w, p, q, r, phi, theta, psi)

    # Inputs: surge force, pitch & yaw moments
    Fx = SX.sym('Fx')
    My = SX.sym('My')
    Mz = SX.sym('Mz')
    u_in = vertcat(Fx, My, Mz)

    # Inertia (diagonal for now)
    #M = diag(SX([m, m, m, Ix, Iy, Iz]))

    # Coriolis matrix for diagonal inertia
    C = coriolis_rb_diag(m[0][0], Ix, Iy, Iz, u, v, w, p, q, r)

    # Damping (linear + quadratic diagonal)
    Dl, Dq, abs_vel = dampening(D_lin, D_quad, u, v, w, p, q, r)

    # Generalized input vector tau: map [Fx, My, Mz] to 6x1 wrench
    # tau = [Fx, 0, 0, 0, My, Mz]^T
    tau = vertcat(Fx, 0.0, 0.0, 0.0, My, Mz)


    # 6DOF body dynamics: nu_dot = M^{-1} (tau - C*nu - (Dl + Dq*|nu|) nu - g)
    nu = vertcat(u, v, w, p, q, r)
    D_eff_times_nu = (Dl @ nu) + (Dq @ (abs_vel * nu))  # elementwise: (Dq*|nu|) * nu
    rhs_nu = SX(6, 1)
    rhs_nu = inv(m) @ (tau - C @ nu - D_eff_times_nu)

    # Kinematics: eta_dot = T(eta) * omega
    T = euler_kinematics_T(phi, theta)
    eta_dot = T @ vertcat(p, q, r)

    xdot = vertcat(rhs_nu, eta_dot)

    model = {
        "name": "auv_model",
        "x": x,
        #"xdot": xdot,
        "u": u_in,
        "f_expl_expr": xdot,   # explicit ODE f(x,u)
        # implicit form (optional in acados): f_impl = xdot - f(x,u)
        #"f_impl_expr": xdot - xdot
    }
    return model

if __name__ == "__main__":
    # Quick smoke test
    m=[[6,0,0,0,0,0,],
        [0,5,0,0,0,0,],
        [0,0,4,0,0,0,],
        [0,0,0,3,0,0,],
        [0,0,0,0,5,0,],
        [0,0,0,0,0,9]]
    D_lin=[[6,0,0,0,0,0,],
        [0,5,0,0,0,0,],
        [0,0,4,0,0,0,],
        [0,0,0,3,0,0,],
        [0,0,0,0,5,0,],
        [0,0,0,0,0,9]]
    D_quad=[[6,0,0,0,0,0,],
        [0,5,0,0,0,0,],
        [0,0,4,0,0,0,],
        [0,0,0,3,0,0,],
        [0,0,0,0,5,0,],
        [0,0,0,0,0,9]]
    mdl = make_auv_model(m,D_lin,D_quad)
    print("Model:", mdl["name"])
    print("x dim:", mdl["x"].numel(), "u dim:", mdl["u"].numel())
