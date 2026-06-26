#pragma once

#include <chrono>
#include <memory>

#include <vortex/utils/types.hpp>

#include "reference_filter_dp_quat/lib/waypoint_follower.hpp"

namespace vortex::guidance {

enum class WaypointStatus {
    idle,
    running,
    succeeded,
    canceled,
    rejected,
};

struct WaypointGuidanceManagerConfig {
    ReferenceFilterParams filter_params{};

    std::chrono::milliseconds time_step{10};

    bool altitude_control_enabled{false};

    // filtered = alpha * previous + (1 - alpha) * new_measurement
    double altitude_low_pass_alpha{0.9};
};

struct GuidanceReference {
    vortex::utils::types::Pose pose{};
    vortex::utils::types::Twist twist{};

    WaypointStatus status{WaypointStatus::idle};

    bool active{false};
    bool just_completed{false};
};

class WaypointGuidanceManager {
public:
    explicit WaypointGuidanceManager(
        const WaypointGuidanceManagerConfig& config);

    void set_pose(const vortex::utils::types::Pose& pose);

    void set_twist(const vortex::utils::types::Twist& twist);

    /**
     * Updates filtered DVL altitude.
     *
     * Values <= 0.0 are ignored, matching the old ROS node.
     */
    void set_altitude(double altitude_m);

    /**
     * Start a waypoint trajectory.
     *
     * If another waypoint is already running, the current filter state is
     * preserved and the follower retargets smoothly.
     */
    [[nodiscard]]
    WaypointStatus submit_waypoint(
        vortex::utils::types::Waypoint waypoint,
        double convergence_threshold);

    /**
     * Cancel the active waypoint without changing the latest vehicle state.
     */
    void cancel_waypoint();

    /**
     * Clear active guidance state.
     *
     * This replaces the old "mission/wipe" callback.
     */
    void reset();

    /**
     * Advance the reference generator exactly one time step.
     *
     * Call once from the unified vehicle-control loop.
     */
    [[nodiscard]]
    GuidanceReference tick();

    [[nodiscard]]
    WaypointStatus status() const noexcept;

    [[nodiscard]]
    bool active() const noexcept;

    [[nodiscard]]
    bool altitude_valid() const noexcept;

    [[nodiscard]]
    double altitude() const noexcept;

    [[nodiscard]]
    const vortex::utils::types::Pose& current_goal() const;

private:
    [[nodiscard]]
    bool prepare_altitude_goal(vortex::utils::types::Waypoint& waypoint);

    void recreate_follower();

    WaypointGuidanceManagerConfig config_{};

    std::unique_ptr<WaypointFollower> follower_{};

    vortex::utils::types::Pose current_pose_{};
    vortex::utils::types::Twist current_twist_{};

    double current_altitude_{0.0};
    bool altitude_valid_{false};

    bool waypoint_active_{false};

    bool keep_altitude_{false};
    bool require_altitude_convergence_{true};

    double desired_altitude_{0.0};
    double convergence_threshold_{0.1};

    WaypointStatus status_{WaypointStatus::idle};
};

}  // namespace vortex::guidance
