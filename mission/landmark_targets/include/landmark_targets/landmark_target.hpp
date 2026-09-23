#ifndef LANDMARK_TARGETS__LANDMARK_TARGET_HPP_
#define LANDMARK_TARGETS__LANDMARK_TARGET_HPP_

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <optional>
#include <vortex/utils/types.hpp>

namespace vortex::mission {

using vortex::utils::types::Pose;

/**
 * @brief How TargetSpec::offset is interpreted.
 */
enum class OffsetFrame : uint8_t {
    /// Offset in the landmark frame: origin in the object, +X out of its
    /// front, +Z down. Needs a landmark with orientation.
    LANDMARK,
    /// Offset along the odom axes (z < 0 is above). For objects without
    /// orientation. Identical to vortex::utils::waypoints::apply_pose_offset.
    LANDMARK_ODOM_AXES
};

struct TargetSpec {
    /// Offset from the landmark, in the frame given by @p frame.
    Pose offset{};
    OffsetFrame frame{OffsetFrame::LANDMARK_ODOM_AXES};
    /// base_link -> tool, in base_link (from TF). The target position then
    /// applies to the tool (launcher, dropper, camera, gripper) and the
    /// orientation to base_link.
    Eigen::Vector3d tool_arm{Eigen::Vector3d::Zero()};
    /// Compute the target once and never update it (after CommitEstimate).
    bool freeze{false};
    /// Within this distance [m] of the target the node stops updating; the
    /// reference filter and controller hold the last goal.
    double dead_reckoning_distance{0.5};
    /// The landmark not seen for this long [s] before dead reckoning -> LOST.
    double track_loss_timeout_sec{10.0};
    /// A new goal is only sent when the target has moved more than this [m].
    double resend_distance{0.05};
    /// Minimum time [s] between two goals.
    double min_resend_interval_sec{0.3};
};

/// A landmark from the map (converted from LandmarkTrack in the node).
struct MapLandmark {
    int id{-1};
    Pose pose{};
    bool has_orientation{false};
    /// Time [s] of the last measurement, on the same clock as `now`.
    double last_measurement{0.0};
};

/**
 * @brief Odom pose for base_link that puts the tool at landmark + offset.
 *
 * p_base = p_target - R(q_target) * tool_arm, orientation = q_target.
 *
 * @throws std::invalid_argument if the frame is LANDMARK and the landmark has
 * no orientation.
 */
Pose resolve_target(const MapLandmark& landmark, const TargetSpec& spec);

enum class Phase : uint8_t { TRACKING, DEAD_RECKONING, LOST };

struct TargetStep {
    /// Set = send a new WaypointManager goal with this odom pose (base_link).
    std::optional<Pose> send_goal;
    Phase phase{Phase::TRACKING};
};

/**
 * @brief State for one approach to a landmark. Pure logic, no ROS.
 *
 * Call step() periodically. The first step that sees the landmark returns the
 * initial goal. Afterwards a goal is only returned when the target has moved
 * more than resend_distance (and not more often than min_resend_interval_sec).
 * DEAD_RECKONING and LOST are terminal: no goals are returned any more.
 */
class LandmarkTarget {
   public:
    LandmarkTarget(TargetSpec spec, int landmark_id);

    /**
     * @param landmark The landmark with the locked id from the map, if any.
     * @param odom Current vehicle (base_link) pose in odom.
     * @param now Current time [s].
     */
    TargetStep step(const std::optional<MapLandmark>& landmark,
                    const Pose& odom,
                    double now);

    Phase phase() const { return phase_; }
    int landmark_id() const { return landmark_id_; }
    /// The last target that was returned as a goal, if any.
    std::optional<Pose> last_goal() const { return last_goal_; }

   private:
    TargetSpec spec_;
    int landmark_id_;
    Phase phase_{Phase::TRACKING};
    std::optional<Pose> last_goal_;
    double last_goal_time_{0.0};
    std::optional<double> absent_since_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_TARGETS__LANDMARK_TARGET_HPP_
