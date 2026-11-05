#include "roll_guidance/roll_guidance.hpp"

#include <stdexcept>

#include <vortex/utils/math.hpp>


namespace vortex::guidance {

/**
 * @brief Constructs a roll maneuver generator.
 *
 * Initialises the generator with the provided roll parameters
 * and validates that the number of waypoints is positive.
 *
 * @param params Configuration parameters for the roll maneuver.
 * @throws std::invalid_argument If number_of_steps is less than
 *         or equal to zero.
 */
RollManeuverGenerator::RollManeuverGenerator(
    const ROLL::Params& params)
    : params_(params)
{
    if (params_.number_of_steps <= 0) {
        throw std::invalid_argument(
            "ROLL number_of_steps must be positive");
    }
}

/**
 * @brief Generates orientation waypoints for a roll maneuver.
 *
 * Creates a sequence of poses representing incremental rotations
 * around the vehicle's roll axis.
 *
 * The total roll angle is divided equally among the specified
 * number of steps. Each waypoint preserves the initial position
 * while applying an additional rotation to the initial orientation.
 *
 * @param initial_pose Initial position and orientation of the AUV.
 *
 * @return A vector of poses containing the generated roll waypoints.
 *         The initial pose is excluded, and the final waypoint
 *         represents the completed roll maneuver.
 *
 * @note The waypoints are equally spaced in angle, not in time.
 */
std::vector<vortex::utils::types::Pose> RollManeuverGenerator::generate_waypoints(
    const vortex::utils::types::Pose& initial_pose) const {

    std::vector<vortex::utils::types::Pose> waypoints;
    waypoints.reserve(params_.number_of_steps);

    const double angle_per_step = params_.roll_angle / params_.number_of_steps;

    const Eigen::Quaterniond initial_orientation = initial_pose.ori_quaternion();

    for (int i = 1; i <= params_.number_of_steps; ++i) {
        const double angle = i * angle_per_step;

        const Eigen::Quaterniond roll_rotation = vortex::utils::math::euler_to_quat(
                angle, 0.0, 0.0);

        vortex::utils::types::Pose waypoint = initial_pose;

        waypoint.set_ori(initial_orientation * roll_rotation);

        waypoints.push_back(waypoint);
    }

    return waypoints;
}

}  // namespace vortex::guidance