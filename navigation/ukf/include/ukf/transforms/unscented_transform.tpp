#include <array>
#include <cmath>

template <Manifold InputSpace, Manifold OutputSpace, typename Function>
    requires std::invocable<Function, const typename InputSpace::Point&>
UnscentedTransformOutput<typename OutputSpace::Point, OutputSpace::dimension>
UnscentedTransform::transform(
    const Gaussian<typename InputSpace::Point, dimension>& prior_gaussian,
    const Function& f,
    const Eigen::Matrix<double, OutputSpace::dimension, OutputSpace::dimension>&
        noise_covariance_matrix,
    const InputSpace& input_space,
    const OutputSpace& output_space) const {
    static_assert(InputSpace::dimension == dimension);
    constexpr int output_dimension = OutputSpace::dimension;
    using InputPoint = typename InputSpace::Point;
    using OutputPoint = typename OutputSpace::Point;
    using OutputTangent = Eigen::Matrix<double, output_dimension, 1>;

    // TODO: implement ldlt as fallback if llt fails
    const Eigen::Matrix<double, dimension, dimension> L =
        prior_gaussian.covariance.llt().matrixL();

    // sigma points are the mean perturbed by delta in the tangent space,
    // the deltas are kept since the cross covariance needs them
    std::array<Eigen::Matrix<double, dimension, 1>, sigma_point_count> deltas;
    std::array<InputPoint, sigma_point_count> sigma_points;
    deltas[0].setZero();
    sigma_points[0] = prior_gaussian.mean;

    for (int i = 0; i < dimension; i++) {
        deltas[1 + i] = std::sqrt(dimension + lambda_) * L.col(i);
        deltas[1 + dimension + i] = -std::sqrt(dimension + lambda_) * L.col(i);
    }
    for (int i = 1; i < sigma_point_count; i++) {
        sigma_points[i] =
            input_space.composition_plus(prior_gaussian.mean, deltas[i]);
    }

    // push the sigma points through nonlinearity
    std::array<OutputPoint, sigma_point_count> transformed_sigma_points;
    for (int i = 0; i < sigma_point_count; i++) {
        transformed_sigma_points[i] = f(sigma_points[i]);
    }

    // mean, one step in the tangent space around the transformed centre point
    const OutputPoint& y_ref = transformed_sigma_points[0];
    OutputTangent mean_delta = OutputTangent::Zero();
    for (int i = 0; i < sigma_point_count; i++) {
        mean_delta +=
            weights_mean_(i) *
            output_space.composition_minus(transformed_sigma_points[i], y_ref);
    }
    const OutputPoint mean = output_space.composition_plus(y_ref, mean_delta);

    // covariance and cross covariance about the computed mean
    Eigen::Matrix<double, output_dimension, output_dimension> covariance =
        noise_covariance_matrix;
    Eigen::Matrix<double, dimension, output_dimension> cross_covariance =
        Eigen::Matrix<double, dimension, output_dimension>::Zero();
    for (int i = 0; i < sigma_point_count; i++) {
        const OutputTangent difference =
            output_space.composition_minus(transformed_sigma_points[i], mean);
        covariance +=
            weights_covariance_(i) * difference * difference.transpose();
        cross_covariance +=
            weights_covariance_(i) * deltas[i] * difference.transpose();
    }
    covariance = 0.5 * (covariance + covariance.transpose());

    // clang-format off
    return {
        .transformed = {.mean = mean, .covariance = covariance},
        .cross_covariance = cross_covariance
    };
    // clang-format on
}
