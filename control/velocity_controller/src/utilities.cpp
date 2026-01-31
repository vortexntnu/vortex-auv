#include "velocity_controller/utilities.hpp"
#include "Eigen/Dense"

angle quaternion_to_euler_angle(double w, double x, double y, double z){
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
};
angle NED_to_BODY(const angle &a,const State &s){
    //TODO tests for illegal angles
    Eigen::Vector3d q;
    q<<a.phit,a.thetat,a.psit;
    q=NED_to_BODY(q,s);
    return {q(0),q(1),q(2)};
}

Eigen::Vector3d NED_to_BODY(const Eigen::Vector3d &a, const State &s){
    const double phi = s.roll;
    const double theta = s.pitch;
    const double psi = s.yaw;

    // Rotation matrices (right-handed):
    Eigen::Matrix3d Rz, Ry, Rx;
    Rz << std::cos(psi), -std::sin(psi), 0.0,
          std::sin(psi),  std::cos(psi), 0.0,
          0.0,            0.0,           1.0;

    Ry <<  std::cos(theta), 0.0, std::sin(theta),
           0.0,             1.0, 0.0,
          -std::sin(theta), 0.0, std::cos(theta);

    Rx << 1.0, 0.0,           0.0,
          0.0, std::cos(phi), -std::sin(phi),
          0.0, std::sin(phi),  std::cos(phi);

    // R_n_b maps body -> navigation: v_nav = R_n_b * v_body
    Eigen::Matrix3d R_n_b = Rz * Ry * Rx;

    // To get body from navigation (NED->BODY): apply transpose (inverse)
    Eigen::Vector3d v_body = R_n_b.transpose() * a;
    /*
    Eigen::Matrix3d T;
    T<<cos(s.pitch)*cos(s.yaw),cos(s.pitch)*sin(s.yaw),sin(s.pitch),
        cos()
    */
    return v_body;

}
