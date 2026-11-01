#ifndef ROLL_GUIDANCE__ROLL_TRACKER_HPP_
#define ROLL_GUIDANCE__ROLL_TRACKER_HPP_

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace vortex::guidance {

/**
 * @brief Tracks measured rotation around the AUV's initial body X-axis.
 *
 * Accumulates signed roll increments from successive measured
 * quaternions so that complete revolutions remain distinguishable
 * from the initial orientation.
 */
class RollTracker {
   public:
    /**
     * @brief Initialises the tracker at the start of a maneuver.
     *
     * @param initial_orientation Measured initial AUV orientation.
     * @throws std::invalid_argument If the quaternion is invalid.
     */
    void reset(const Eigen::Quaterniond& initial_orientation) {
        initial_orientation_ = normalised(initial_orientation);
        completed_roll_ = 0.0;
        previous_wrapped_roll_ = 0.0;
        initialised_ = true;
    }

    /**
     * @brief Incorporates a new measured orientation.
     *
     * @param measured_orientation Latest measured AUV orientation.
     * @throws std::invalid_argument If the quaternion is invalid,
     * off-axis rotation is excessive, or the measured roll
     * increment is ambiguous.
     *
     * @note Samples must arrive in chronological order.
     */
    void observe(const Eigen::Quaterniond& measured_orientation) {
        if (!initialised_) {
            throw std::logic_error("RollTracker has not been reset");
        }

        const Eigen::Quaterniond current =
            normalised(measured_orientation);

        const Eigen::Quaterniond relative =
            initial_orientation_.conjugate() * current;

        if (std::hypot(relative.x(), relative.w()) < 0.5) {
            throw std::invalid_argument(
                "Attitude too far from initial roll axis");
        }

        const double wrapped_roll = wrap(
            2.0 * std::atan2(relative.x(), relative.w()));

        const double increment =
            wrap(wrapped_roll - previous_wrapped_roll_);

        constexpr double kPi = 3.14159265358979323846;

        if (std::abs(increment) >= kPi / 2.0) {
            throw std::invalid_argument(
                "Roll changed by at least 90 degrees between samples");
        }

        completed_roll_ += increment;
        previous_wrapped_roll_ = wrapped_roll;
    }

    /**
     * @brief Returns the accumulated signed roll.
     * @return Measured roll angle in radians.
     */
    double completed_roll() const {
        return completed_roll_;
    }

    /**
     * @brief Checks whether the requested rotation was measured.
     *
     * @param requested_roll Requested signed rotation in radians.
     * @param tolerance Allowed angular error in radians.
     * @return true when measured and requested roll agree.
     */
    bool completed(double requested_roll, double tolerance) const {
        return initialised_ &&
               std::abs(completed_roll_ - requested_roll) <= tolerance;
    }

    /**
     * @brief Returns whether the tracker has been initialised.
     * @return true after a successful reset.
     */
    bool initialised() const {
        return initialised_;
    }

   private:
    /**
     * @brief Normalises and validates an orientation quaternion.
     *
     * @param q Input quaternion.
     * @return Unit quaternion.
     * @throws std::invalid_argument If q is invalid.
     */
    static Eigen::Quaterniond normalised(
        const Eigen::Quaterniond& q) {

        if (!q.coeffs().allFinite() ||
            !std::isfinite(q.norm()) ||
            q.norm() < 1e-9) {
            throw std::invalid_argument("Invalid orientation quaternion");
        }

        return q.normalised();
    }

    /**
     * @brief Wraps an angular increment to [-pi, pi].
     *
     * @param angle Angular increment in radians.
     * @return Wrapped angular increment in radians.
     */
    static double wrap(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    Eigen::Quaterniond initial_orientation_{
        Eigen::Quaterniond::Identity()};

    double completed_roll_{0.0};
    double previous_wrapped_roll_{0.0};
    bool initialised_{false};
};

}  // namespace vortex::guidance

#endif  // ROLL_GUIDANCE__ROLL_TRACKER_HPP_
