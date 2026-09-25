#include "landmark_server/course_frame.hpp"
#include <cmath>
#include <vortex/utils/math.hpp>

namespace vortex::mission {

namespace {

double yaw_of(const vortex::utils::types::Pose& pose) {
    const Eigen::Quaterniond q = pose.ori_quaternion().normalized();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// Circular mean of angles.
double circular_mean(const std::deque<double>& angles) {
    double s = 0.0;
    double c = 0.0;
    for (const double a : angles) {
        s += std::sin(a);
        c += std::cos(a);
    }
    return std::atan2(s, c);
}

}  // namespace

CourseFrameTracker::CourseFrameTracker(CourseFrameConfig config)
    : config_(std::move(config)) {}

CourseFrameTracker::Result CourseFrameTracker::set_coarse(
    const vortex::utils::types::Pose& start_pose,
    double heading_offset_rad) {
    const Eigen::Vector3d p = start_pose.pos_vector();
    if (!p.allFinite() || !std::isfinite(start_pose.qw) ||
        !std::isfinite(start_pose.qx) || !std::isfinite(start_pose.qy) ||
        !std::isfinite(start_pose.qz)) {
        return {false, "start_pose contains NaN or inf"};
    }
    if (start_pose.ori_quaternion().norm() < 1e-6) {
        return {false, "start_pose has a zero quaternion"};
    }
    if (!std::isfinite(heading_offset_rad)) {
        return {false, "heading_offset_rad is NaN or inf"};
    }

    // Only the four coin flip directions are legal (pi and -pi are the same).
    constexpr double kTol = 1e-3;
    const double wrapped = vortex::utils::math::ssa(heading_offset_rad);
    const bool legal = std::abs(wrapped) < kTol ||
                       std::abs(std::abs(wrapped) - M_PI_2) < kTol ||
                       std::abs(std::abs(wrapped) - M_PI) < kTol;
    if (!legal) {
        return {false, "heading_offset_rad must be 0, +-pi/2 or pi, got " +
                           std::to_string(heading_offset_rad)};
    }

    start_yaw_ = vortex::utils::math::ssa(yaw_of(start_pose) + wrapped);
    through_yaw_ = start_yaw_;
    origin_ = p.head<2>();
    estimates_.clear();
    deviation_deg_ = 0.0;
    deviation_warning_pending_ = false;
    status_ = CourseFrameStatus::COARSE;
    return {true, "course frame set (coarse)"};
}

void CourseFrameTracker::reset() {
    status_ = CourseFrameStatus::UNSET;
    origin_.setZero();
    through_yaw_ = 0.0;
    start_yaw_ = 0.0;
    deviation_deg_ = 0.0;
    deviation_warning_pending_ = false;
    estimates_.clear();
}

double CourseFrameTracker::yaw_std() const {
    if (estimates_.size() < 2) {
        return 0.0;
    }
    std::deque<double> yaws;
    for (const auto& e : estimates_) {
        yaws.push_back(e.yaw);
    }
    const double mean = circular_mean(yaws);
    double sum = 0.0;
    for (const double y : yaws) {
        const double d = vortex::utils::math::ssa(y - mean);
        sum += d * d;
    }
    return std::sqrt(sum / static_cast<double>(yaws.size()));
}

void CourseFrameTracker::add_gate_estimate(const Eigen::Vector2d& gate_center,
                                           double gate_yaw) {
    if (status_ != CourseFrameStatus::COARSE || !gate_center.allFinite() ||
        !std::isfinite(gate_yaw)) {
        return;
    }
    const double through = vortex::utils::math::ssa(gate_yaw + M_PI);
    estimates_.push_back({gate_center, through});
    while (static_cast<int>(estimates_.size()) >
           config_.gate_lock_consistent_estimates) {
        estimates_.pop_front();
    }

    if (static_cast<int>(estimates_.size()) <
        config_.gate_lock_consistent_estimates) {
        return;
    }
    const double max_std = config_.gate_lock_max_yaw_std_deg * M_PI / 180.0;
    if (yaw_std() > max_std) {
        return;  // not consistent yet; keep sliding the window
    }

    std::deque<double> yaws;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (const auto& e : estimates_) {
        yaws.push_back(e.yaw);
        center += e.center;
    }
    origin_ = center / static_cast<double>(estimates_.size());
    through_yaw_ = circular_mean(yaws);
    deviation_deg_ =
        std::abs(vortex::utils::math::ssa(through_yaw_ - start_yaw_)) * 180.0 /
        M_PI;
    deviation_warning_pending_ =
        deviation_deg_ > config_.warn_start_vs_gate_deg;
    status_ = CourseFrameStatus::GATE_LOCKED;
}

void CourseFrameTracker::apply_correction(const Eigen::Isometry3d& delta) {
    if (status_ == CourseFrameStatus::UNSET) {
        return;
    }
    const double dyaw = std::atan2(delta.linear()(1, 0), delta.linear()(0, 0));
    const auto move = [&](const Eigen::Vector2d& p) {
        return (delta * Eigen::Vector3d(p.x(), p.y(), 0.0)).head<2>().eval();
    };
    origin_ = move(origin_);
    through_yaw_ = vortex::utils::math::ssa(through_yaw_ + dyaw);
    start_yaw_ = vortex::utils::math::ssa(start_yaw_ + dyaw);
    for (auto& e : estimates_) {
        e.center = move(e.center);
        e.yaw = vortex::utils::math::ssa(e.yaw + dyaw);
    }
}

bool CourseFrameTracker::take_deviation_warning() {
    const bool pending = deviation_warning_pending_;
    deviation_warning_pending_ = false;
    return pending;
}

Eigen::Vector2d CourseFrameTracker::to_course(
    const Eigen::Vector2d& odom_xy) const {
    return ::vortex::mission::to_course(
        CourseFrame{origin_, through_yaw_, CourseState::COARSE}, odom_xy);
}

Eigen::Vector2d CourseFrameTracker::from_course(
    const Eigen::Vector2d& course_xy) const {
    return ::vortex::mission::from_course(
        CourseFrame{origin_, through_yaw_, CourseState::COARSE}, course_xy);
}

bool CourseFrameTracker::position_allowed(
    const Eigen::Vector3d& odom_position) const {
    if (status_ == CourseFrameStatus::UNSET) {
        return true;
    }
    const Eigen::Vector2d c = to_course(odom_position.head<2>());
    const LaneBox& box = status_ == CourseFrameStatus::GATE_LOCKED
                             ? config_.after_gate
                             : config_.before_gate;
    return box.contains(c.x(), c.y());
}

}  // namespace vortex::mission
