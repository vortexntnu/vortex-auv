template <SensorModel Sensor>
UpdateOutput<Sensor::dimension> UnscentedKalmanFilter::update(
    const StateGaussian& predicted,
    const typename Sensor::Point& z_current,
    const ImuInput& input,
    const Sensor& measurement_model) const {
    constexpr int M = Sensor::dimension;

    // same as in predict, the input is captured
    auto h = [&](const State& x) { return measurement_model.h(x, input); };

    const Eigen::Matrix<double, M, M> R =
        measurement_model.R(predicted.mean, input);

    const auto transform_output = unscented_transform_.transform(
        predicted, h, R, motion_model_, measurement_model);

    const auto& z_predicted = transform_output.transformed.mean;
    const Eigen::Matrix<double, M, M>& S =
        transform_output.transformed.covariance;
    const Eigen::Matrix<double, StateIndex::size, M>& cross_covariance =
        transform_output.cross_covariance;

    const Eigen::Matrix<double, M, 1> innovation =
        measurement_model.composition_minus(z_current, z_predicted);

    // clang-format off
    const Eigen::Matrix<double, StateIndex::size, M> W = S.ldlt().solve(cross_covariance.transpose()).transpose();
    const State x_updated = motion_model_.composition_plus(predicted.mean, W * innovation);
    const Eigen::Matrix15d P_updated = predicted.covariance - W * S * W.transpose();

    return UpdateOutput<M>{
        .posterior = {
            .mean = x_updated,
            .covariance = 0.5 * (P_updated + P_updated.transpose())
        },
        .innovation = {
            .mean = innovation,
            .covariance = S
        }
    };
    // clang-format on
}
