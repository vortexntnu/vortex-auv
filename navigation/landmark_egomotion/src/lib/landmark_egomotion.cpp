#include "landmark_egomotion/lib/landmark_egomotion.hpp"
#include <algorithm>
#include <cmath>

LandmarkESKF::LandmarkESKF(const EskfParams& params) : ESKF(params) {
    use_vo_ = false;
    disable_gating_ = false;
    consecutive_rejects_ = 0;

    have_anchor_ = false;
    q_nav_vo_ = Eigen::Quaterniond::Identity();
    p_nav_vo_ = Eigen::Vector3d::Zero();

    last_stamp_ = 0.0;
    have_prev_ = false;
    prev_stamp_ = 0.0;
    prev_p_nav_ = Eigen::Vector3d::Zero();

    vo_cfg_ = {};
}

Eigen::Vector3d LandmarkESKF::compute_quaternion_error(
    const Eigen::Quaterniond& q_meas,
    const Eigen::Quaterniond& q_est) {
    Eigen::Quaterniond q_err = (q_meas * q_est.inverse()).normalized();

    if (q_err.w() < 0.0) {
        q_err.coeffs() = -q_err.coeffs();
    }

    const double w = q_err.w();
    const Eigen::Vector3d v(q_err.x(), q_err.y(), q_err.z());
    const double v_norm = v.norm();

    if (v_norm < 1e-10) {
        return 2.0 * v;
    }

    const double theta = 2.0 * std::atan2(v_norm, w);

    return (theta / v_norm) * v;
}

Eigen::Vector3d LandmarkESKF::log_quat(const Eigen::Quaterniond& q) {
    const double w = std::clamp(static_cast<double>(q.w()), -1.0, 1.0);
    const double theta = 2.0 * std::acos(w);
    const double s = std::sqrt(std::max(0.0, 1.0 - w * w));

    if (s < 1e-9) {
        return Eigen::Vector3d(2 * q.x(), 2 * q.y(), 2 * q.z());
    }

    return Eigen::Vector3d(q.x(), q.y(), q.z()) * (theta / s);
}

Eigen::Quaterniond LandmarkESKF::exp_rotvec(const Eigen::Vector3d& w) {
    const double theta = w.norm();
    if (theta < 1e-9) {
        return Eigen::Quaterniond(1.0, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z())
            .normalized();
    }
    const Eigen::Vector3d a = w / theta;
    const double s = std::sin(0.5 * theta);

    return Eigen::Quaterniond(std::cos(0.5 * theta), a.x() * s, a.y() * s,
                              a.z() * s);
}

double LandmarkESKF::angle_dist(const Eigen::Quaterniond& a,
                                const Eigen::Quaterniond& b) {
    const double d = std::abs(a.dot(b));

    return 2.0 * std::acos(std::clamp(d, -1.0, 1.0));
}

double LandmarkESKF::compute_nis(const Eigen::VectorXd& innovation,
                                 const Eigen::MatrixXd& S) {
    return innovation.transpose() * S.inverse() * innovation;
}

void LandmarkESKF::sw_add(double stamp,
                          const Eigen::Vector3d& p,
                          const Eigen::Quaterniond& q) {
    sw_buf_.push_back({stamp, p, q});
}

void LandmarkESKF::sw_prune(double now) {
    while (!sw_buf_.empty() &&
           (now - sw_buf_.front().stamp) > vo_cfg_.sw_max_age) {
        sw_buf_.pop_front();
    }
    while (static_cast<int>(sw_buf_.size()) > vo_cfg_.sw_window_size) {
        sw_buf_.pop_front();
    }
}

bool LandmarkESKF::sw_estimate(Eigen::Vector3d& p_out,
                               Eigen::Quaterniond& q_out) {
    if (sw_buf_.empty())
        return false;

    q_out = sw_buf_.back().quat;
    p_out = Eigen::Vector3d::Zero();

    double gate = vo_cfg_.sw_gate_deg * M_PI / 180.0;
    const double huber = vo_cfg_.sw_huber_deg * M_PI / 180.0;

    for (int iter = 0; iter < 5; ++iter) {
        Eigen::Vector3d sum_p = Eigen::Vector3d::Zero();
        Eigen::Vector3d sum_w = Eigen::Vector3d::Zero();
        double w_total = 0.0;

        for (const auto& s : sw_buf_) {
            const double dist = angle_dist(q_out, s.quat);
            if (dist > gate)
                continue;

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

        if (w_total < 1e-9)
            return false;

        p_out = sum_p / w_total;

        const Eigen::Vector3d dw = sum_w / w_total;
        q_out = (q_out * exp_rotvec(dw)).normalized();

        if (dw.norm() < 1e-4)
            break;
    }

    return true;
}

void LandmarkESKF::landmark_vel_update_(const Eigen::Vector3d& v_meas_nav,
                                        const Eigen::Matrix3d& Rv) {
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    const Eigen::Matrix15d P = current_error_state_.covariance;
    const Eigen::Vector3d y = v_meas_nav - current_nom_state_.vel;

    const Eigen::Matrix3d S = H * P * H.transpose() + Rv;
    const double nis = compute_nis(y, S);

    const double gate = disable_gating_ ? 1e9 : vo_cfg_.nis_gate_vel;
    if (nis > gate) {
        return;
    }

    Eigen::Matrix<double, 15, 3> K = P * H.transpose() * S.inverse();
    K *= vo_cfg_.vel_alpha;

    current_error_state_.set_from_vector(K * y);

    Eigen::Matrix<double, 15, 3> K_full = P * H.transpose() * S.inverse();
    Eigen::Matrix15d I_KH = Eigen::Matrix15d::Identity() - K_full * H;
    current_error_state_.covariance =
        I_KH * P * I_KH.transpose() + K_full * Rv * K_full.transpose();

    injection_and_reset();
}

void LandmarkESKF::landmark_update(const LandmarkMeasurement& landmark_meas) {
    if (!use_vo_) {
        return;
    }

    if (last_stamp_ > 0.0) {
        const double gap = landmark_meas.stamp_ - last_stamp_;
        if (gap > vo_cfg_.dropout_timeout) {
            have_anchor_ = false;
            have_prev_ = false;
            consecutive_rejects_ = 0;
            sw_buf_.clear();
        }
    }
    last_stamp_ = landmark_meas.stamp_;

    const bool need_reset = (consecutive_rejects_ >= vo_cfg_.rejects_limit);

    if (!have_anchor_ || need_reset) {
        q_nav_vo_ = (current_nom_state_.quat * landmark_meas.quat.inverse())
                        .normalized();
        p_nav_vo_ = current_nom_state_.pos - q_nav_vo_ * landmark_meas.pos;

        have_anchor_ = true;
        have_prev_ = false;
        consecutive_rejects_ = 0;
        sw_buf_.clear();
        return;
    }

    // Transform VO measurement to navigation frame
    Eigen::Vector3d p_nav = p_nav_vo_ + q_nav_vo_ * landmark_meas.pos;
    Eigen::Quaterniond q_nav = (q_nav_vo_ * landmark_meas.quat).normalized();

    if (vo_cfg_.use_sw) {
        sw_add(landmark_meas.stamp_, p_nav, q_nav);
        sw_prune(landmark_meas.stamp_);

        Eigen::Vector3d p_smooth;
        Eigen::Quaterniond q_smooth;
        if (sw_estimate(p_smooth, q_smooth)) {
            p_nav = p_smooth;
            q_nav = q_smooth;
        }
    }

    if (have_prev_) {
        const double dt = landmark_meas.stamp_ - prev_stamp_;
        if (dt > vo_cfg_.dt_min && dt < vo_cfg_.dt_max) {
            const Eigen::Vector3d v_meas_nav = (p_nav - prev_p_nav_) / dt;

            Eigen::Matrix3d Rp = landmark_meas.R.topLeftCorner<3, 3>();
            Eigen::Matrix3d Rv = 2.0 * Rp / (dt * dt);

            const double v_floor_sq = vo_cfg_.vel_floor * vo_cfg_.vel_floor;
            for (int i = 0; i < 3; ++i) {
                Rv(i, i) = std::max(Rv(i, i), v_floor_sq);
            }

            landmark_vel_update_(v_meas_nav, Rv);
        }
    }
    prev_stamp_ = landmark_meas.stamp_;
    prev_p_nav_ = p_nav;
    have_prev_ = true;

    Eigen::Matrix<double, 6, 1> y;

    // Position innovation
    y.head<3>() = p_nav - current_nom_state_.pos;
    y.segment<3>(3) = compute_quaternion_error(q_nav, current_nom_state_.quat);

    // Measurement Jacobian
    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

    // Measurement noise floors
    Eigen::Matrix<double, 6, 6> R = landmark_meas.R;

    const double p_floor_sq = vo_cfg_.pos_floor * vo_cfg_.pos_floor;
    const double a_floor_sq = vo_cfg_.att_floor * vo_cfg_.att_floor;

    for (int i = 0; i < 3; ++i) {
        R(i, i) = std::max(R(i, i), p_floor_sq);
    }
    for (int i = 3; i < 6; ++i) {
        R(i, i) = std::max(R(i, i), a_floor_sq);
    }

    // Innovation covariance and NIS
    const Eigen::Matrix15d P = current_error_state_.covariance;
    const Eigen::Matrix<double, 6, 6> S = H * P * H.transpose() + R;
    const double nis = compute_nis(Eigen::VectorXd(y), Eigen::MatrixXd(S));

    const double gate = disable_gating_ ? 1e9 : vo_cfg_.nis_gate_pose;

    if (nis > gate) {
        consecutive_rejects_++;
        return;
    }

    Eigen::Matrix<double, 15, 6> K = P * H.transpose() * S.inverse();
    current_error_state_.set_from_vector(K * y);

    // Joseph form covariance update
    Eigen::Matrix15d I_KH = Eigen::Matrix15d::Identity() - K * H;
    current_error_state_.covariance =
        I_KH * P * I_KH.transpose() + K * R * K.transpose();

    injection_and_reset();
    consecutive_rejects_ = 0;
}

void LandmarkESKF::set_vo_enabled(bool enabled) {
    use_vo_ = enabled;
}

void LandmarkESKF::set_nis_gating_enabled(bool enabled) {
    disable_gating_ = !enabled;
}

void LandmarkESKF::handle_marker_switch(const Eigen::Vector3d& p_base,
                                        const Eigen::Quaterniond& q_base) {
    q_nav_vo_ = (current_nom_state_.quat * q_base.inverse()).normalized();
    p_nav_vo_ = current_nom_state_.pos - q_nav_vo_ * p_base;
    sw_buf_.clear();
}

void LandmarkESKF::force_anchor_reset() {
    have_anchor_ = false;
    have_prev_ = false;
    consecutive_rejects_ = 0;
    sw_buf_.clear();
}

int LandmarkESKF::get_consecutive_rejects() const {
    return consecutive_rejects_;
}

bool LandmarkESKF::is_anchor_valid() const {
    return have_anchor_;
}

void LandmarkESKF::set_vo_config(const VoConfig& cfg) {
    vo_cfg_ = cfg;
}
