#include "eskf/eskf.hpp"
#include <spdlog/spdlog.h>
#include <functional>
#include <iterator>
#include <unsupported/Eigen/MatrixFunctions>
#include <vortex/utils/math.hpp>
#include "eskf/typedefs.hpp"

static double compute_nis(const Eigen::Vector3d& innovation,
                   const Eigen::Matrix3d& S) {
    Eigen::Matrix3d S_inv = S.inverse();
    return innovation.transpose() * S_inv * innovation;
}

static double compute_nis(const Eigen::VectorXd& innovation,
                           const Eigen::MatrixXd& S) {
    return innovation.transpose() * S.inverse() * innovation;
}

ESKF::ESKF(const EskfParams& params) : Q_(params.Q) {
    current_error_state_.covariance = params.P;

    use_vo_ = false; 
    disable_gating_ = false;
    consecutive_rejects_ = 0;

    have_anchor_ = false;
    q_nav_vo_ = Eigen::Quaterniond::Identity();
    p_nav_vo_ = Eigen::Vector3d::Zero();

    last_stamp_= 0.0;
    have_prev_ = false;
    prev_stamp_ = 0.0;
    prev_p_nav_ = Eigen::Vector3d::Zero();

    vo_cfg_ = {};

    nis_ = 0.0;
}

static Eigen::Vector3d compute_quaternion_error(const Eigen::Quaterniond& q_meas,
                                                 const Eigen::Quaterniond& q_est) {
    Eigen::Quaterniond q_err = (q_meas * q_est.inverse()).normalized();
    
    // Ensure shortest path by keeping w >= 0
    if (q_err.w() < 0.0) {
        q_err.coeffs() = -q_err.coeffs();
    }
    
    // Convert to rotation vector using log map
    const double w = q_err.w();
    const Eigen::Vector3d v(q_err.x(), q_err.y(), q_err.z());
    const double v_norm = v.norm();
    
    if (v_norm < 1e-10) {
        return 2.0 * v;
    }
    
    const double theta = 2.0 * std::atan2(v_norm, w);
    return (theta / v_norm) * v;
}

Eigen::Vector3d ESKF::log_quat(const Eigen::Quaterniond& q) {
    const double w = std::clamp(static_cast<double>(q.w()), -1.0, 1.0);
    const double theta = 2.0 * std::acos(w);
    const double s = std::sqrt(std::max(0.0, 1.0 - w * w));

    if (s < 1e-9) {
        return Eigen::Vector3d(2 * q.x(), 2 * q.y(), 2 * q.z());
    }
    return Eigen::Vector3d(q.x(), q.y(), q.z()) * (theta / s);
}

Eigen::Quaterniond ESKF::exp_rotvec(const Eigen::Vector3d& w) {
    const double theta = w.norm();
    if (theta < 1e-9) {
        return Eigen::Quaterniond(1.0, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z()).normalized();
    }
    const Eigen::Vector3d a = w / theta;
    const double s = std::sin(0.5 * theta);
    return Eigen::Quaterniond(std::cos(0.5 * theta), a.x() * s, a.y() * s, a.z() * s);
}

double ESKF::angle_dist(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
    const double d = std::abs(a.dot(b));
    return 2.0 * std::acos(std::clamp(d, -1.0, 1.0));
}


void ESKF::sw_add(double stamp, const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    sw_buf_.push_back({stamp, p, q});
}

void ESKF::sw_prune(double now) {
    while (!sw_buf_.empty() && (now - sw_buf_.front().stamp) > vo_cfg_.sw_max_age) {
        sw_buf_.pop_front();
    }
    while (static_cast<int>(sw_buf_.size()) > vo_cfg_.sw_window_size) {
        sw_buf_.pop_front();
    }
}

bool ESKF::sw_estimate(Eigen::Vector3d& p_out, Eigen::Quaterniond& q_out) {
    if (sw_buf_.empty()) return false;

    q_out = sw_buf_.back().quat;
    p_out = Eigen::Vector3d::Zero();

    double gate = vo_cfg_.sw_gate_deg * M_PI / 180.0;
    const double huber = vo_cfg_.sw_huber_deg * M_PI / 180.0;

    // Iterative mean
    for (int iter = 0; iter < 5; ++iter) {
        Eigen::Vector3d sum_p = Eigen::Vector3d::Zero();
        Eigen::Vector3d sum_w = Eigen::Vector3d::Zero();
        double w_total = 0.0;

        for (const auto& s : sw_buf_) {
            const double dist = angle_dist(q_out, s.quat);
            if (dist > gate) continue;

            const Eigen::Vector3d w_i = log_quat(q_out.inverse() * s.quat);

            double weight = 1.0;
            if (huber > 0.0) {
                const double a = w_i.norm();
                if (a > huber && a > 1e-12) {
                    weight = huber / a;
                }
            }

            sum_p += s.pos * weight;
            sum_w += w_i * weight;
            w_total += weight;
        }

        if (w_total < 1e-9) return false;

        p_out = sum_p / w_total;

        const Eigen::Vector3d dw = sum_w / w_total;
        q_out = (q_out * exp_rotvec(dw)).normalized();

        if (dw.norm() < 1e-4) break;
    }

    return true;
}

std::pair<Eigen::Matrix18d, Eigen::Matrix18d> ESKF::van_loan_discretization(
    const Eigen::Matrix18d& A_c,
    const Eigen::Matrix18x12d& G_c,
    const double dt) {
    Eigen::Matrix18d GQG_T = G_c * Q_ * G_c.transpose();
    Eigen::Matrix36d vanLoanMat = Eigen::Matrix36d::Zero();

    vanLoanMat.topLeftCorner<18, 18>() = -A_c;
    vanLoanMat.topRightCorner<18, 18>() = GQG_T;
    vanLoanMat.bottomRightCorner<18, 18>() = A_c.transpose();

    Eigen::Matrix36d vanLoanExp = (vanLoanMat * dt).exp();

    Eigen::Matrix18d V1 = vanLoanExp.bottomRightCorner<18, 18>().transpose();
    Eigen::Matrix18d V2 = vanLoanExp.topRightCorner<18, 18>();

    Eigen::Matrix18d A_d = V1;
    Eigen::Matrix18d GQG_d = A_d * V2;

    return {A_d, GQG_d};
}

Eigen::Matrix3x19d ESKF::calculate_hx() {
    Eigen::Matrix3x19d Hx = Eigen::Matrix3x19d::Zero();

    Eigen::Quaterniond q = current_nom_state_.quat.normalized();
    Eigen::Matrix3d R_bn = q.toRotationMatrix();

    Eigen::Vector3d v_n = current_nom_state_.vel;

    // Correct derivative w.r.t velocity (nominal state: v_n)
    Hx.block<3, 3>(0, 3) = R_bn.transpose();

    // Derivative w.r.t quaternion (nominal state: q)
    // Compute partial derivative w.r.t quaternion directly:
    double qw = q.w();
    Eigen::Vector3d q_vec(q.x(), q.y(), q.z());
    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 4> dhdq;
    dhdq.col(0) = 2 * (qw * v_n + q_vec.cross(v_n));
    dhdq.block<3, 3>(0, 1) =
        2 * (q_vec.dot(v_n) * I3 + q_vec * v_n.transpose() -
             v_n * q_vec.transpose() -
             qw * vortex::utils::math::get_skew_symmetric_matrix(v_n));

    // Assign quaternion derivative (3x4 block at columns 6:9)
    Hx.block<3, 4>(0, 6) = dhdq;

    return Hx;
}

Eigen::Matrix3x18d ESKF::calculate_h_jacobian() {
    Eigen::Matrix19x18d x_delta = Eigen::Matrix19x18d::Zero();
    x_delta.block<6, 6>(0, 0) = Eigen::Matrix6d::Identity();
    x_delta.block<4, 3>(6, 6) =
        vortex::utils::math::get_transformation_matrix_attitude_quat(
            current_nom_state_.quat);
    x_delta.block<9, 9>(10, 9) = Eigen::Matrix9d::Identity();

    Eigen::Matrix3x18d H = calculate_hx() * x_delta;
    return H;
}

Eigen::Vector3d ESKF::calculate_h() {
    Eigen::Vector3d h;
    Eigen::Matrix3d R_bn =
        current_nom_state_.quat.normalized().toRotationMatrix().transpose();

    h = R_bn * current_nom_state_.vel;
    // 0.027293, 0.028089, 0.028089, 0.00255253, 0.00270035, 0.00280294,
    return h;
}

void ESKF::nominal_state_discrete(const ImuMeasurement& imu_meas,
                                  const double dt) {
    Eigen::Vector3d acc =
        current_nom_state_.quat.normalized().toRotationMatrix() *
            (imu_meas.accel - current_nom_state_.accel_bias) +
        current_nom_state_.gravity;
    Eigen::Vector3d gyro = (imu_meas.gyro - current_nom_state_.gyro_bias) * dt;

    current_nom_state_.pos = current_nom_state_.pos +
                             current_nom_state_.vel * dt + 0.5 * dt * dt * acc;
    current_nom_state_.vel = current_nom_state_.vel + dt * acc;

    current_nom_state_.quat =
        (current_nom_state_.quat *
         vortex::utils::math::eigen_vector3d_to_quaternion(gyro));
    current_nom_state_.quat.normalize();

    current_nom_state_.gyro_bias = current_nom_state_.gyro_bias;
    current_nom_state_.accel_bias = current_nom_state_.accel_bias;
    current_nom_state_.gravity = current_nom_state_.gravity;
}

void ESKF::error_state_prediction(const ImuMeasurement& imu_meas,
                                  const double dt) {
    Eigen::Matrix3d R = current_nom_state_.quat.normalized().toRotationMatrix();
    Eigen::Vector3d acc = (imu_meas.accel - current_nom_state_.accel_bias);
    Eigen::Vector3d gyro = (imu_meas.gyro - current_nom_state_.gyro_bias);

    Eigen::Matrix18d A_c = Eigen::Matrix18d::Zero();
    A_c.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(3, 6) =
        -R * vortex::utils::math::get_skew_symmetric_matrix(acc);
    A_c.block<3, 3>(6, 6) =
        -vortex::utils::math::get_skew_symmetric_matrix(gyro);
    A_c.block<3, 3>(3, 9) = -R;
    A_c.block<3, 3>(9, 9) = -Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(12, 12) = -Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();

    Eigen::Matrix18x12d G_c = Eigen::Matrix18x12d::Zero();
    G_c.block<3, 3>(3, 0) = -R;
    G_c.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();
    G_c.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    G_c.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    Eigen::Matrix18d A_d, GQG_d;
    std::tie(A_d, GQG_d) = van_loan_discretization(A_c, G_c, dt);

    StateEuler next_error_state;
    current_error_state_.covariance =
        A_d * current_error_state_.covariance * A_d.transpose() + GQG_d;
}

void ESKF::measurement_update(const DvlMeasurement& dvl_meas) {
    Eigen::Matrix3x18d H = calculate_h_jacobian();
    Eigen::Matrix18d P = current_error_state_.covariance;
    Eigen::Matrix3d R = dvl_meas.cov;

    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix18x3d K = P * H.transpose() * S.inverse();
    Eigen::Vector3d innovation = dvl_meas.vel - calculate_h();

    nis_ = compute_nis(innovation, S);
    current_error_state_.set_from_vector(K * innovation);

    Eigen::Matrix18d I_KH = Eigen::Matrix18d::Identity() - K * H;
    current_error_state_.covariance =
        I_KH * P * I_KH.transpose() +
        K * R * K.transpose();  // Used joseph form for more stable calculations
}

void ESKF::injection_and_reset() {
    current_nom_state_.pos = current_nom_state_.pos + current_error_state_.pos;
    current_nom_state_.vel = current_nom_state_.vel + current_error_state_.vel;
    current_nom_state_.quat = current_nom_state_.quat *
                              vortex::utils::math::eigen_vector3d_to_quaternion(
                                  current_error_state_.euler);
    current_nom_state_.quat.normalize();
    current_nom_state_.gyro_bias =
        current_nom_state_.gyro_bias + current_error_state_.gyro_bias;
    current_nom_state_.accel_bias =
        current_nom_state_.accel_bias + current_error_state_.accel_bias;
    current_nom_state_.gravity =
        current_nom_state_.gravity + current_error_state_.gravity;

    Eigen::Matrix18d G = Eigen::Matrix18d::Identity();

    current_error_state_.covariance =
        G * current_error_state_.covariance * G.transpose();
    current_error_state_.set_from_vector(Eigen::Vector18d::Zero());
}

void ESKF::imu_update(const ImuMeasurement& imu_meas, const double dt) {
    nominal_state_discrete(imu_meas, dt);
    error_state_prediction(imu_meas, dt);
}

void ESKF::dvl_update(const DvlMeasurement& dvl_meas) {
    // for testing the visual odom
    return;
    
    //measurement_update(dvl_meas);
    //injection_and_reset();
}

void ESKF::landmark_vel_update_(const Eigen::Vector3d& v_meas_nav,
                               const Eigen::Matrix3d& Rv) {
    Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    const Eigen::Matrix18d P = current_error_state_.covariance;
    const Eigen::Vector3d y = v_meas_nav - current_nom_state_.vel;

    const Eigen::Matrix3d S = H * P * H.transpose() + Rv;
    const double nis = compute_nis(y, S);

    const double gate = disable_gating_ ? 1e9 : vo_cfg_.nis_gate_vel;
    if (nis > gate) {
        spdlog::debug("VO velocity rejected: NIS={:.2f}", nis);
        return;
    }

    Eigen::Matrix<double, 18, 3> K = P * H.transpose() * S.inverse();
    K *= vo_cfg_.vel_alpha;

    current_error_state_.set_from_vector(K * y);

    Eigen::Matrix<double, 18, 3> K_full = P * H.transpose() * S.inverse();
    Eigen::Matrix18d I_KH = Eigen::Matrix18d::Identity() - K_full * H;
    current_error_state_.covariance = I_KH * P * I_KH.transpose() + K_full * Rv * K_full.transpose();

    injection_and_reset();
}

void ESKF::landmark_update(const VisualMeasurement& visual_meas) {
    if (!use_vo_) { return; }

    if (last_stamp_ > 0.0) {
        const double gap = visual_meas.stamp_ - last_stamp_;
        if (gap > vo_cfg_.dropout_timeout_sec) {
            spdlog::info("VO dropout detected ({:.2f}s), resetting anchor", gap);
            have_anchor_ = false;
            have_prev_ = false;
            consecutive_rejects_ = 0;
            sw_buf_.clear();
        }
    }
    last_stamp_ = visual_meas.stamp_;

    const bool need_reset = (consecutive_rejects_ >= vo_cfg_.rejects_limit);

    if (!have_anchor_ || need_reset) {
        if (need_reset) {
            spdlog::warn("Smart anchor reset: {} consecutive rejections", consecutive_rejects_);
        }
        
        q_nav_vo_ = (current_nom_state_.quat * visual_meas.quat.inverse()).normalized();
        p_nav_vo_ = current_nom_state_.pos - q_nav_vo_ * visual_meas.pos;

        have_anchor_ = true;
        have_prev_ = false;
        consecutive_rejects_ = 0;
        sw_buf_.clear();
        
        spdlog::info("VO anchor initialized at pos=[{:.2f}, {:.2f}, {:.2f}]",
                    p_nav_vo_.x(), p_nav_vo_.y(), p_nav_vo_.z());
        return;
    }

    // transform VO measurement to navigation frame
    Eigen::Vector3d p_nav = p_nav_vo_ + q_nav_vo_ * visual_meas.pos;
    Eigen::Quaterniond q_nav =
        (q_nav_vo_ * visual_meas.quat).normalized();

    if (vo_cfg_.use_sw) {
        sw_add(visual_meas.stamp_, p_nav, q_nav);
        sw_prune(visual_meas.stamp_);

        Eigen::Vector3d p_smooth;
        Eigen::Quaterniond q_smooth;
        if (sw_estimate(p_smooth, q_smooth)) {
            p_nav = p_smooth;
            q_nav = q_smooth;
        }
    }

    if (have_prev_) {
        const double dt = visual_meas.stamp_ - prev_stamp_;
        if (dt > vo_cfg_.dt_min && dt < vo_cfg_.dt_max) {
            const Eigen::Vector3d v_meas_nav = (p_nav - prev_p_nav_) / dt;

            Eigen::Matrix3d Rp = visual_meas.R.topLeftCorner<3, 3>();
            Eigen::Matrix3d Rv = 2.0 * Rp / (dt * dt);

            const double v_floor_sq = vo_cfg_.vel_floor * vo_cfg_.vel_floor;
            for (int i = 0; i < 3; ++i) {
                Rv(i, i) = std::max(Rv(i, i), v_floor_sq);
            }

            landmark_vel_update_(v_meas_nav, Rv);
        }
    }
    prev_stamp_ = visual_meas.stamp_;
    prev_p_nav_ = p_nav;
    have_prev_ = true;

    Eigen::Matrix<double, 6, 1> y;

    // position innovation
    y.head<3>() = p_nav - current_nom_state_.pos;
    y.segment<3>(3) = compute_quaternion_error(q_nav, current_nom_state_.quat);

    // measurement Jacobian
    Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // position
    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();  // attitude error-state

    // measurement noise floors
    Eigen::Matrix<double, 6, 6> R = visual_meas.R;
    
    const double p_floor_sq = vo_cfg_.pos_floor * vo_cfg_.pos_floor;
    const double a_floor_sq = vo_cfg_.att_floor * vo_cfg_.att_floor;
    
    for (int i = 0; i < 3; ++i) {
        R(i, i) = std::max(R(i, i), p_floor_sq);
    }
    for (int i = 3; i < 6; ++i) {
        R(i, i) = std::max(R(i, i), a_floor_sq);
    }

    // innovation covariance and NIS
    const Eigen::Matrix18d P = current_error_state_.covariance;
    const Eigen::Matrix<double, 6, 6> S = H * P * H.transpose() + R;
    const double nis = compute_nis(Eigen::VectorXd(y), Eigen::MatrixXd(S));

    const double gate = disable_gating_ ? 1e9 : vo_cfg_.nis_gate_pose;

    if (nis > gate) {
        consecutive_rejects_++;
        
        spdlog::warn("VO pose REJECTED: NIS={:.1f} (gate={:.1f}), "
                    "pos_err={:.3f}m, att_err={:.1f}deg, rejects={}",
                    nis, gate,
                    y.head<3>().norm(),
                    y.segment<3>(3).norm() * 180.0 / M_PI,
                    consecutive_rejects_);
        return;
    }

    Eigen::Matrix<double, 18, 6> K = P * H.transpose() * S.inverse();
    current_error_state_.set_from_vector(K * y);

    // Joseph form covariance update
    Eigen::Matrix18d I_KH = Eigen::Matrix18d::Identity() - K * H;
    current_error_state_.covariance =
        I_KH * P * I_KH.transpose() + K * R * K.transpose();

    injection_and_reset();
    consecutive_rejects_ = 0;
}

void ESKF::set_vo_enabled(bool enabled) {
    use_vo_ = enabled;
    spdlog::info("VO gating {}", enabled ? "ENABLED" : "DISABLED");
}

void ESKF::set_nis_gating_enabled(bool enabled) {
    disable_gating_ = !enabled;
    spdlog::info("NIS gating {}", enabled ? "ENABLED" : "DISABLED");
}

void ESKF::handle_marker_switch(const Eigen::Vector3d& p_base, const Eigen::Quaterniond& q_base) {
    q_nav_vo_ = (current_nom_state_.quat * q_base.inverse()).normalized();
    p_nav_vo_ = current_nom_state_.pos - q_nav_vo_ * p_base;
    sw_buf_.clear();
    spdlog::info("Marker switch: anchor updated");
}

void ESKF::force_anchor_reset() {
    have_anchor_ = false;
    have_prev_ = false;
    consecutive_rejects_ = 0;
    sw_buf_.clear();
}

int ESKF::get_consecutive_rejects() const {
    return consecutive_rejects_;
}

bool ESKF::is_anchor_valid() const {
    return have_anchor_;
}

void ESKF::set_vo_config(const VoConfig& cfg) {
    vo_cfg_ = cfg;
}