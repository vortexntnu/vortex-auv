#ifndef ROLL_GUIDANCE__ROLL_GUIDANCE_HPP_
#define ROLL_GUIDANCE__ROLL_GUIDANCE_HPP_

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <vortex/utils/types.hpp>

namespace vortex::guidance {

namespace ROLL {

/**
 * @struct Params
 * @brief Parameters for the roll manoeuver.
 */
struct Params {
    /** @brief Total roll angle in radians. */
    double roll_angle{2.0 * M_PI};

    /** @brief Number of orientation waypoints. */
    int number_of_steps{8};

    /** @brief Distance from the gate at which the maneuver starts, in meters. */
    double trigger_distance{3.0};
};

}  // namespace ROLL

/**
 * @brief Generates orientation waypoints for an AUV roll maneuver.
 *
 * The generator creates a sequence of poses that rotate the AUV
 * around its local X-axis while maintaining its initial position.
 */
class RollManeuverGenerator {
   public:
    /**
     * @brief Constructs the ROLL maneuver generator.
     * @param params Parameters defining the maneuver.
     */
    explicit RollManeuverGenerator(const ROLL::Params& params);

    /**
     * @brief Generates cumulative orientation waypoints for a full roll.
     * @param initial_pose AUV pose at the start of the maneuver.
     * @return Ordered poses representing the roll maneuver.
     */
    std::vector<vortex::utils::types::Pose> generate_waypoints(
        const vortex::utils::types::Pose& initial_pose) const;

   private:
    ROLL::Params params_;
};

}  // namespace vortex::guidance

#endif  // ROLL_GUIDANCE__ROLL_GUIDANCE_HPP_