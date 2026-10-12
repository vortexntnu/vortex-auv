/**
 * @file ukf.hpp
 * @brief Contains the UnscentedKalmanFilter class, with predict and update
 * steps built on the unscented transform.
 */

#ifndef UKF__FILTERS__UKF_HPP_
#define UKF__FILTERS__UKF_HPP_

#include "ukf/models/manifold.hpp"
#include "ukf/models/strapdown_ins.hpp"
#include "ukf/transforms/unscented_transform.hpp"
#include "ukf/typedefs.hpp"

/**
 * @brief Unscented Kalman filter on the strapdown INS manifold. The filter is
 * stateless, the estimate is passed in and returned by predict and update.
 */
class UnscentedKalmanFilter {
   public:
    /**
     * @brief Constructor for the UnscentedKalmanFilter class.
     * @param motion_model The strapdown INS motion model.
     * @param transform_config The scaling parameters of the unscented
     * transform.
     * @throws std::invalid_argument if the transform config is invalid.
     */
    UnscentedKalmanFilter(const StrapdownINS3D& motion_model,
                          const TransformConfig& transform_config);

    /**
     * @brief Propagates the estimate one IMU step through the motion model.
     * @param current The current state estimate.
     * @param input The IMU measurement, including the time step.
     * @return The predicted state estimate.
     */
    StateGaussian predict(const StateGaussian& current,
                          const ImuInput& input) const;

    /**
     * @brief Updates the estimate with a measurement.
     * @tparam Sensor The measurement model, satisfying SensorModel.
     * @param predicted The predicted state estimate.
     * @param z_current The measurement.
     * @param input The IMU measurement, passed on to the measurement model.
     * @param measurement_model The measurement model of the sensor.
     * @return The posterior state estimate and the innovation with its
     * covariance.
     */
    template <SensorModel Sensor>
    UpdateOutput<Sensor::dimension> update(
        const StateGaussian& predicted,
        const typename Sensor::Point& z_current,
        const ImuInput& input,
        const Sensor& measurement_model) const;

   private:
    StrapdownINS3D motion_model_;
    UnscentedTransform unscented_transform_;
};

#include "ukf/filters/ukf.tpp"

#endif  // UKF__FILTERS__UKF_HPP_
