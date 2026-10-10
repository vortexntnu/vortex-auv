#include "ukf/filters/ukf.hpp"

UnscentedKalmanFilter::UnscentedKalmanFilter(
    const StrapdownINS3D& motion_model,
    const TransformConfig& transform_config)
    : motion_model_(motion_model), unscented_transform_(transform_config) {}

StateGaussian UnscentedKalmanFilter::predict(const StateGaussian& current,
                                             const ImuInput& input) const {
    // the transform wants a function of x only, so the input is captured
    auto f = [&](const State& x) { return motion_model_.f(x, input); };

    const Eigen::Matrix15d Q = motion_model_.Q(current.mean, input);

    const auto transform_output = unscented_transform_.transform(
        current, f, Q, motion_model_, motion_model_);

    // the cross covariance is only needed by a smoother
    return transform_output.transformed;
}
