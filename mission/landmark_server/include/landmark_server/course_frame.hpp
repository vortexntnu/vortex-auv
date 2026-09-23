#ifndef LANDMARK_SERVER__COURSE_FRAME_HPP_
#define LANDMARK_SERVER__COURSE_FRAME_HPP_

#include <deque>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vortex/utils/types.hpp>
#include "landmark_server/class_config.hpp"

namespace vortex::mission {

enum class CourseFrameStatus : uint8_t { UNSET, COARSE, GATE_LOCKED };

/**
 * @brief The course frame (TF nautilus/course): origin at the gate, x through
 * the gate, y to the left (NED, z down). ROS-free.
 *
 * - UNSET: nothing known; no frame exists.
 * - COARSE: from the start pose and the coin flip angle (set_coarse).
 * - GATE_LOCKED: moved to the gate centre once the gate yaw has been
 *   consistent for N estimates (add_gate_estimate).
 *
 * The lane bounds are boxes in this frame: generous before the gate is
 * locked, tight after. They keep the neighbouring lane's gate and pipes out
 * of the map. Odom X is the heading the ESKF started with, not the course
 * direction, so nothing here uses fixed odom coordinates.
 */
class CourseFrameTracker {
   public:
    struct Result {
        bool success{false};
        std::string message;
    };

    explicit CourseFrameTracker(CourseFrameConfig config = {});

    /**
     * @brief Set the coarse frame from the start pose and coin flip angle.
     * Also resets a locked frame (new run).
     * @param start_pose Vehicle pose in odom at start.
     * @param heading_offset_rad Course direction relative to the vehicle:
     * 0, +-pi/2 or pi.
     */
    Result set_coarse(const vortex::utils::types::Pose& start_pose,
                      double heading_offset_rad);

    /// Back to UNSET (mission/wipe).
    void reset();

    /**
     * @brief Add a gate estimate. @p gate_yaw is the yaw of GATE_WHOLE, which
     * points out of the front towards the start, so the course direction is
     * gate_yaw + pi. Ignored unless the state is COARSE.
     */
    void add_gate_estimate(const Eigen::Vector2d& gate_center, double gate_yaw);

    CourseFrameStatus status() const { return status_; }
    Eigen::Vector2d origin() const { return origin_; }
    double through_yaw() const { return through_yaw_; }
    /// Spread of the latest gate yaw estimates [rad] (0 with < 2 estimates).
    double yaw_std() const;
    int consistent_estimates() const {
        return static_cast<int>(estimates_.size());
    }
    /// |gate direction - start direction| [deg]; 0 before locking.
    double start_vs_gate_deviation_deg() const { return deviation_deg_; }
    /// True once when locking found a deviation over the warning limit.
    bool take_deviation_warning();

    /// Course coordinates of an odom position (x through the gate, y left).
    Eigen::Vector2d to_course(const Eigen::Vector2d& odom_xy) const;
    Eigen::Vector2d from_course(const Eigen::Vector2d& course_xy) const;

    /// Whether an odom position is inside the lane bounds. Always true while
    /// the frame is UNSET.
    bool position_allowed(const Eigen::Vector3d& odom_position) const;

   private:
    CourseFrameConfig config_;
    CourseFrameStatus status_{CourseFrameStatus::UNSET};
    Eigen::Vector2d origin_{Eigen::Vector2d::Zero()};
    double through_yaw_{0.0};
    double start_yaw_{0.0};
    double deviation_deg_{0.0};
    bool deviation_warning_pending_{false};

    struct Estimate {
        Eigen::Vector2d center;
        double yaw;
    };
    std::deque<Estimate> estimates_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__COURSE_FRAME_HPP_
