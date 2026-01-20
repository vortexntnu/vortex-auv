#include "valve_egomotion/valve_egomotion.hpp"

#include <algorithm>
#include <cmath>

namespace valve_egomotion {

SlidingWindowSO3Mean::SlidingWindowSO3Mean(Config cfg) : cfg_(cfg) {}

void SlidingWindowSO3Mean::clear() {
    buf_.clear();
}

void SlidingWindowSO3Mean::addSample(const rclcpp::Time& stamp,
                                     const tf2::Transform& T_ref_base,
                                     double weight) {
    buf_.push_back(Sample{stamp, T_ref_base, weight});
}

void SlidingWindowSO3Mean::prune(const rclcpp::Time& now) {
    while (!buf_.empty() &&
           (now - buf_.front().stamp).seconds() > cfg_.win_age_sec) {
        buf_.pop_front();
    }
    while (static_cast<int>(buf_.size()) > cfg_.win_size) {
        buf_.pop_front();
    }
}

bool SlidingWindowSO3Mean::estimate(const rclcpp::Time& now,
                                    tf2::Transform& T_est,
                                    std::array<double, 36>& cov6x6) const {
    (void)now;
    if (buf_.empty())
        return false;

    // Init around the newest sample rotation; translation init at origin
    tf2::Quaternion q_avg = buf_.back().T_ref_base.getRotation();
    tf2::Vector3 p_avg(0, 0, 0);

    double gate = cfg_.gate_threshold_deg * M_PI / 180.0;
    const double huber = cfg_.huber_threshold_deg * M_PI / 180.0;

    if (cfg_.use_mad_gate && buf_.size() >= 5) {
        std::vector<double> dists;
        dists.reserve(buf_.size());
        for (const auto& s : buf_) {
            dists.push_back(angularDist(q_avg, s.T_ref_base.getRotation()));
        }
        const double med = median(dists);
        for (auto& d : dists)
            d = std::abs(d - med);
        const double mad = median(dists);
        gate = std::min(gate, med + 3.0 * mad);
    }

    // Robust mean on SO(3) using log/exp in the tangent space at q_avg
    for (int i = 0; i < 5; ++i) {
        tf2::Vector3 sum_p(0, 0, 0);
        tf2::Vector3 sum_w(0, 0, 0);
        double w_total = 0.0;

        for (const auto& s : buf_) {
            const tf2::Quaternion q_i = s.T_ref_base.getRotation();
            const double dist = angularDist(q_avg, q_i);
            if (dist > gate)
                continue;

            const tf2::Vector3 w_i = logMapSO3(q_avg.inverse() * q_i);

            double w = s.weight;
            if (huber > 0.0) {
                const double a = w_i.length();
                if (a > huber && a > 1e-12)
                    w *= (huber / a);
            }

            sum_p += s.T_ref_base.getOrigin() * w;
            sum_w += w_i * w;
            w_total += w;
        }

        if (w_total < 1e-9)
            return false;

        p_avg = sum_p / w_total;

        const tf2::Vector3 dw = sum_w / w_total;
        q_avg = (q_avg * expMapSO3(dw)).normalized();

        if (dw.length() < 1e-4)
            break;
    }

    T_est.setOrigin(p_avg);
    T_est.setRotation(q_avg);

    // Diagonal covariance proxy
    cov6x6.fill(0.0);
    double inliers = 0.0;

    for (const auto& s : buf_) {
        const tf2::Quaternion q_i = s.T_ref_base.getRotation();
        if (angularDist(q_avg, q_i) > gate)
            continue;

        const tf2::Vector3 dp = s.T_ref_base.getOrigin() - p_avg;
        const tf2::Vector3 dq = logMapSO3(q_avg.inverse() * q_i);

        cov6x6[0] += dp.x() * dp.x();
        cov6x6[7] += dp.y() * dp.y();
        cov6x6[14] += dp.z() * dp.z();
        cov6x6[21] += dq.x() * dq.x();
        cov6x6[28] += dq.y() * dq.y();
        cov6x6[35] += dq.z() * dq.z();

        inliers += 1.0;
    }

    const double denom = std::max(1.0, inliers);
    for (double& v : cov6x6)
        v /= denom;

    const double pos_floor = 1e-6;  
    const double rot_floor = 1e-6;  
    cov6x6[0] += pos_floor;
    cov6x6[7] += pos_floor;
    cov6x6[14] += pos_floor;
    cov6x6[21] += rot_floor;
    cov6x6[28] += rot_floor;
    cov6x6[35] += rot_floor;

    return true;
}

tf2::Vector3 SlidingWindowSO3Mean::logMapSO3(const tf2::Quaternion& q) {
    const double w = std::clamp(static_cast<double>(q.getW()), -1.0, 1.0);
    const double th = 2.0 * std::acos(w);
    const double s = std::sqrt(std::max(0.0, 1.0 - w * w));

    // Small-angle: log(q) ≈ 2*v where q ≈ [v, 1]
    if (s < 1e-9) {
        return tf2::Vector3(2 * q.getX(), 2 * q.getY(), 2 * q.getZ());
    }
    return tf2::Vector3(q.getX(), q.getY(), q.getZ()) * (th / s);
}

tf2::Quaternion SlidingWindowSO3Mean::expMapSO3(const tf2::Vector3& w) {
    const double th = w.length();
    if (th < 1e-9) {
        return tf2::Quaternion(0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z(), 1.0)
            .normalized();
    }
    const tf2::Vector3 a = w / th;
    return tf2::Quaternion(a.x() * std::sin(0.5 * th),
                           a.y() * std::sin(0.5 * th),
                           a.z() * std::sin(0.5 * th), std::cos(0.5 * th));
}

double SlidingWindowSO3Mean::angularDist(const tf2::Quaternion& q1,
                                         const tf2::Quaternion& q2) {
    // Use abs(dot) to account for q and -q representing same rotation
    const double d = std::abs(static_cast<double>(q1.dot(q2)));
    return 2.0 * std::acos(std::clamp(d, -1.0, 1.0));
}

template <typename T>
T SlidingWindowSO3Mean::median(std::vector<T> v) {
    const size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

template double SlidingWindowSO3Mean::median<double>(std::vector<double>);

}  // namespace valve_egomotion