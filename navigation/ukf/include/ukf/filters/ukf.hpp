#ifndef UKF__FILTERS__UKF_HPP_
#define UKF__FILTERS__UKF_HPP_

#include "ukf/models/manifold.hpp"
#include "ukf/models/strapdown_ins.hpp"
#include "ukf/transforms/unscented_transform.hpp"
#include "ukf/typedefs.hpp"

class UnscentedKalmanFilter {
   public:
    UnscentedKalmanFilter(const StrapdownINS3D& motion_model,
                          const TransformConfig& transform_config);

    StateGaussian predict(const StateGaussian& current,
                          const ImuInput& input) const;

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
