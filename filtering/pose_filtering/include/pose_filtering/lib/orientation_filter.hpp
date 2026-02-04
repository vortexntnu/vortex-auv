#ifndef POSE_FILTERING__LIB__ORIENTATION_FILTER_HPP_
#define POSE_FILTERING__LIB__ORIENTATION_FILTER_HPP_

#include <Eigen/Dense>
#include <vector>
#include <vortex_filtering/vortex_filtering.hpp>
#include "typedefs.hpp"

namespace vortex::filtering {

/**
 * @brief Orientation filter implementing prediction and PDAF update on SO(3).
 *
 * The filter represents the orientation mean as a quaternion and keeps
 * an error-state Gaussian in the 3D tangent space. Measurement updates
 * are performed by mapping quaternion residuals into the tangent space
 * (log map), performing a PDAF-style weighted update, and applying the
 * update via the exponential map back to the quaternion manifold.
 */
class OrientationFilter {
   public:
    /**
     * @brief Construct an OrientationFilter
     * @param config Configuration struct
     */
    explicit OrientationFilter(const OrientationFilterConfig& config);

    /**
     * @brief Run a single filter step (prediction + update) for orientation.
     * @param measurements Quaternion measurements associated with the
     *        current track
     * @param dt Time step in seconds used for prediction / covariance growth
     * @param state OrientationState instance that will be updated with the
     *        new orientation mean and error-state
     */
    void step(const std::vector<Eigen::Quaterniond>& measurements,
              double dt,
              OrientationState& state);

   private:
    /**
     * @brief Map a quaternion to the so(3) tangent vector (log map).
     * @param q_in Quaternion representing the relative rotation
     * @return Vector in R^3 representing the axis * angle
     */
    Eigen::Vector3d so3_log_quat(const Eigen::Quaterniond& q_in);

    /**
     * @brief Map a tangent vector back to a quaternion (exp map).
     * @param rvec Tangent vector (axis * angle)
     * @return Quaternion corresponding to the rotation
     */
    Eigen::Quaterniond so3_exp_quat(const Eigen::Vector3d& rvec);

    DynMod dyn_mod_;

    SensorMod sensor_mod_;

    vortex::filter::PDAF<DynMod, SensorMod>::Config pdaf_config_;
};

}  // namespace vortex::filtering

#endif  // POSE_FILTERING__LIB__ORIENTATION_FILTER_HPP_
