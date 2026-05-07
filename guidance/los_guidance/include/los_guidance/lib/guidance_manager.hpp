/**
 * @file guidance_manager.hpp
 * @brief The LosGuidanceStateManager class executes LOS guidance logic,
 * manages LOS method selection, and tracks goal feasibility and
 * progress.
 */
#ifndef LOS_GUIDANCE__LIB__GUIDANCE_MANAGER_HPP_
#define LOS_GUIDANCE__LIB__GUIDANCE_MANAGER_HPP_

#include <yaml-cpp/yaml.h>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <vortex_msgs/action/guidance_waypoint.hpp>
#include "los_guidance/lib/adaptive_los.hpp"
#include "los_guidance/lib/integral_los.hpp"
#include "los_guidance/lib/proportional_los.hpp"
#include "los_guidance/lib/types.hpp"
#include "los_guidance/lib/vector_field_los.hpp"

namespace vortex::guidance::los {

class LosGuidanceStateManager {
   public:
    /**
     * @brief Constructs a LosGuidanceStateManager object and loads LOS
     * guidance configuration from the specified YAML file.
     * @param yaml_file_path Path to the YAML configuration file.
     */
    explicit LosGuidanceStateManager(const std::string& yaml_file_path);

    /**
     * @brief Initializes the adaptive LOS guidance module from configuration.
     * @param config YAML configuration node containing adaptive LOS parameters.
     */
    void set_adaptive_los_guidance(YAML::Node config);

    /**
     * @brief Initializes the proportional LOS guidance module from
     * configuration.
     * @param config YAML configuration node containing proportional LOS
     * parameters.
     */
    void set_proportional_los_guidance(YAML::Node config);

    /**
     * @brief Initializes the integral LOS guidance module from configuration.
     * @param config YAML configuration node containing integral LOS parameters.
     */
    void set_integral_los_guidance(YAML::Node config);

    /**
     * @brief Initializes the vector field LOS guidance module from
     * configuration.
     * @param config YAML configuration node containing vector field LOS
     * parameters.
     */
    void set_vector_field_guidance(YAML::Node config);

    /**
     * @brief Parses common guidance parameters shared by all LOS methods.
     * @param common_config YAML node containing common guidance parameters.
     */
    void parse_common_config(const YAML::Node& common_config);

    /**
     * @brief Checks if the given LOS guidance goal is feasible based on the
     * current state.
     * @param goal The goal to check for feasibility.
     * @return true if the goal is feasible, false otherwise.
     */
    bool is_goal_feasible(
        std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal>
            goal);

    /**
     * @brief Checks if the LOS guidance goal has been missed based on the
     * current state.
     * @return true if the goal is missed, false otherwise.
     */
    bool is_goal_missed();

    /**
     * @brief Checks if the LOS guidance goal has been reached.
     * @param tolerance Distance tolerance for goal convergence.
     * @return true if the goal is reached, false otherwise.
     */
    bool is_goal_reached(double tolerance);

    /**
     * @brief Updates the current waypoint.
     * @param new_wp New waypoint to navigate to.
     */
    void update_waypoint(const types::Point& new_wp);

    /**
     * @brief Updates the current vehicle position.
     * @param position Current position of the vehicle.
     */
    void update_position(const types::Point& position);

    /**
     * @brief Updates the current yaw angle.
     * @param yaw Current yaw angle in radians.
     */
    void update_yaw(double yaw);

    /**
     * @brief Sets the active LOS guidance method.
     * @param method The LOS method to use.
     */
    void set_los_method(types::ActiveLosMethod method);

    /**
     * @brief Initializes a new goal.
     * @param new_wp New waypoint to navigate to.
     */
    void initialize_goal(const types::Point& new_wp);

    /**
     * @brief Calculates LOS guidance outputs based on current state.
     * @return types::Outputs Calculated guidance outputs.
     */
    types::Outputs calculate_outputs();

    /**
     * @brief Gets the maximum pitch angle limit.
     * @return Maximum pitch angle in radians.
     */
    double get_max_pitch_angle() const;

    /**
     * @brief Gets the current yaw angle.
     * @return Current yaw angle in radians.
     */
    double get_current_yaw() const;

    /**
     * @brief Gets the desired surge velocity.
     * @return Desired surge velocity.
     */
    double get_u_desired() const;

   private:
    types::Inputs path_inputs_{};
    double u_desired_{};
    double goal_reached_tol_{};
    double max_pitch_angle_{};
    double current_yaw_{};
    double time_step_s_{};
    types::ActiveLosMethod method_{};
    double nearest_been_to_goal_{std::numeric_limits<double>::max()};
    double time_since_nearest_goal_s_{};
    double missed_goal_distance_margin_{};
    double missed_goal_timeout_s_{};
    bool has_active_segment_{false};

    std::unique_ptr<AdaptiveLOSGuidance> adaptive_los_{};
    std::unique_ptr<IntegralLOSGuidance> integral_los_{};
    std::unique_ptr<ProportionalLOSGuidance> proportional_los_{};
    std::unique_ptr<VectorFieldLOSGuidance> vector_field_los_{};

    mutable std::mutex mutex_;
};

}  // namespace vortex::guidance::los

#endif  // LOS_GUIDANCE__LIB__GUIDANCE_MANAGER_HPP_
