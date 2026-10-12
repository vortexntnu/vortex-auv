/**
 * @file unscented_transform.hpp
 * @brief Contains the UnscentedTransform class, which propagates Gaussians
 * on manifolds through nonlinear functions.
 */

#ifndef UKF__TRANSFORMS__UNSCENTED_TRANSFORM_HPP_
#define UKF__TRANSFORMS__UNSCENTED_TRANSFORM_HPP_

#include <concepts>
#include "ukf/models/manifold.hpp"
#include "ukf/typedefs.hpp"

/**
 * @brief The unscented transform, propagating a Gaussian on a manifold
 * through a nonlinear function using sigma points in the tangent space.
 */
class UnscentedTransform {
   public:
    static constexpr int dimension = StateIndex::size;
    static constexpr int sigma_point_count = 2 * dimension + 1;

    /**
     * @brief Constructor for the UnscentedTransform class, computes the
     * sigma point weights.
     * @param transform_config The alpha, beta and kappa scaling parameters.
     * @throws std::invalid_argument if alpha is not positive or if
     * dimension + lambda is not positive.
     */
    explicit UnscentedTransform(const TransformConfig& transform_config);

    /**
     * @brief Transforms a Gaussian through a nonlinear function. The
     * transformed covariance includes the noise, the cross covariance does
     * not.
     * @tparam InputSpace The manifold of the prior Gaussian.
     * @tparam OutputSpace The manifold of the function output.
     * @param prior_gaussian The Gaussian going in.
     * @param f The function to transform through.
     * @param noise_covariance_matrix Noise added to the transformed
     * covariance.
     * @param input_space Provides the composition operators of the input.
     * @param output_space Provides the composition operators of the output.
     * @return The transformed Gaussian and the cross covariance between the
     * input and the output.
     */
    template <Manifold InputSpace, Manifold OutputSpace, typename Function>
        requires std::invocable<Function, const typename InputSpace::Point&>
    UnscentedTransformOutput<typename OutputSpace::Point,
                             OutputSpace::dimension>
    transform(
        const Gaussian<typename InputSpace::Point, dimension>& prior_gaussian,
        const Function& f,
        const Eigen::Matrix<double,
                            OutputSpace::dimension,
                            OutputSpace::dimension>& noise_covariance_matrix,
        const InputSpace& input_space,
        const OutputSpace& output_space) const;

   private:
    Eigen::Matrix<double, sigma_point_count, 1> weights_mean_;
    Eigen::Matrix<double, sigma_point_count, 1> weights_covariance_;
    double lambda_;
};

#include "ukf/transforms/unscented_transform.tpp"

#endif  // UKF__TRANSFORMS__UNSCENTED_TRANSFORM_HPP_
