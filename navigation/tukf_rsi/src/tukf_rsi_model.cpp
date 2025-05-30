#include "tukf_rsi/tukf_model.hpp"
#include "tukf_rsi/tukf_rsi_utils.hpp"
#include "tukf_rsi/typedefs.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


Eigen::Matrix4x3d tranfromation_matrix(const Eigen::Quaterniond& q) {
    Eigen::Matrix4x3d T;
    T << -q.x(), -q.y(), -q.z(),
            q.w(), -q.z(),  q.y(),
            q.z(),  q.w(), -q.x(),
            -q.y(),  q.x(),  q.w();
    return T;
}

Eigen::Matrix6d M_rb(const Eigen::Vector<double, 9>& inertia_vec) {
    const double mass = 30.0; 
    const Eigen::Matrix3d I_rb = Eigen::Map<const Eigen::Matrix3d>(inertia_vec.data());
    const Eigen::Vector3d r_b_bg(0.01, 0.0, 0.02);
    
    Eigen::Matrix6d M = Eigen::Matrix6d::Zero();
    M.block<3,3>(0,0) = mass * Eigen::Matrix3d::Identity();
    M.block<3,3>(0,3) = -mass * skewSymmetric(r_b_bg);
    M.block<3,3>(3,0) = mass * skewSymmetric(r_b_bg);
    M.block<3,3>(3,3) = I_rb;
    
    return M;
}

Eigen::Matrix6d M_a(const Eigen::Vector<double, 6>& a_mass) {
    Eigen::Matrix6d Ma = Eigen::Matrix6d::Zero();
    Ma.block<3,3>(0,0) = a_mass.head<3>().asDiagonal();
    Ma.block<3,3>(3,3) = a_mass.tail<3>().asDiagonal();
    return Ma;
}

Eigen::Matrix6d C_rb(const Eigen::Vector<double, 9>& inertia_vec, const Eigen::Vector3d& w) {
    const double mass = 30.0;
    const Eigen::Vector3d r_b_bg(0.01, 0.0, 0.02);
    const Eigen::Matrix3d I_rb = Eigen::Map<const Eigen::Matrix3d>(inertia_vec.data());
    
    Eigen::Matrix6d C = Eigen::Matrix6d::Zero();
    C.block<3,3>(3,3) = -skewSymmetric(I_rb * w);
    C.block<3,3>(0,3) = -mass * skewSymmetric(w) * skewSymmetric(r_b_bg);
    C.block<3,3>(3,0) = mass * skewSymmetric(r_b_bg) * skewSymmetric(w);
    
    return C;
}

Eigen::Matrix6d C_a(const Eigen::Vector<double, 6>& a_mass, const Eigen::Vector3d& w, const Eigen::Vector3d& v) {
    Eigen::Matrix6d Ca = Eigen::Matrix6d::Zero();
    const Eigen::Matrix3d A11 = a_mass.head<3>().asDiagonal();
    const Eigen::Matrix3d A22 = a_mass.tail<3>().asDiagonal();
    
    Ca.block<3,3>(0,3) = -skewSymmetric(A11 * v);
    Ca.block<3,3>(3,0) = -skewSymmetric(A11 * v);
    Ca.block<3,3>(3,3) = -skewSymmetric(A22 * w);
    
    return Ca;
}

Eigen::Matrix6d D_linear(const Eigen::Vector<double, 6>& d) {
    Eigen::Matrix6d D = Eigen::Matrix6d::Zero();
    D.block<3,3>(0,0) = -d.head<3>().asDiagonal();
    D.block<3,3>(3,3) = -d.tail<3>().asDiagonal();
    return D;
}

Eigen::Vector6d G_eta(const Eigen::Vector<double, 4>& g_params, const Eigen::Quaterniond& q) {
    const double Mx = g_params[1], My = g_params[2], Mz = g_params[3];
    const Eigen::Matrix3d R = q.toRotationMatrix();
    
    Eigen::Vector6d G = Eigen::Vector6d::Zero();
    G(3) = -My*R(2,2) + Mz*R(1,2);
    G(4) = -Mz*R(0,2) + Mx*R(2,2);
    G(5) = -Mx*R(1,2) + My*R(0,2);
    
    return G;
}

AUVState F_dynamics(const AUVState& state, double dt, const Eigen::Vector6d& u) {
    const Eigen::Matrix6d Mrb = M_rb(state.inertia);
    const Eigen::Matrix6d Ma = M_a(state.added_mass);
    const Eigen::Matrix6d Mtotal = Mrb + Ma;
    
    const Eigen::Matrix6d Crb = C_rb(state.inertia, state.angular_velocity);
    const Eigen::Matrix6d Ca = C_a(state.added_mass, state.angular_velocity, state.velocity);
    const Eigen::Matrix6d Ctotal = Crb + Ca;
    
    const Eigen::Matrix6d Dl = D_linear(state.damping);
    const Eigen::Vector6d G = G_eta(state.g_eta, state.orientation);
    
    Eigen::Vector6d nu;
    nu << state.velocity, state.angular_velocity;
    
    AUVState sd;
    sd.position = state.orientation.toRotationMatrix() * state.velocity;
    sd.orientation = tranfromation_matrix(state.orientation) * state.angular_velocity;
    
    Eigen::Vector6d Nu = Mtotal.inverse() * (u - Ctotal*nu - Dl*nu - G);
    sd.velocity = Nu.head<3>();
    sd.angular_velocity = Nu.tail<3>();
    
    sd.inertia.setZero(); 
    sd.added_mass.setZero();
    sd.damping.setZero(); 
    sd.g_eta.setZero();
    
    AUVState ns;
    ns.position = state.position + sd.position * dt;
    ns.orientation = state.orientation * Eigen::Quaterniond(1, 0.5*dt*sd.orientation.x(), 
                                                           0.5*dt*sd.orientation.y(), 
                                                           0.5*dt*sd.orientation.z());
    ns.orientation.normalize();
    ns.velocity = state.velocity + sd.velocity * dt;
    ns.angular_velocity = state.angular_velocity + sd.angular_velocity * dt;
    
    ns.inertia = state.inertia;
    ns.added_mass = state.added_mass;
    ns.damping = state.damping;
    ns.g_eta = state.g_eta;
    
    return ns;
}
