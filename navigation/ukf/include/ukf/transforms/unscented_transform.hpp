#ifndef UKF__TRANSFORMS__UNSCENTED_TRANSFORM_HPP_
#define UKF__TRANSFORMS__UNSCENTED_TRANSFORM_HPP_

#include <concepts>
#include "ukf/models/manifold.hpp"
#include "ukf/typedefs.hpp"

class UnscentedTransform {
   public:
    static constexpr int dimension = StateIndex::size;
    static constexpr int sigma_point_count = 2 * dimension + 1;

    explicit UnscentedTransform(const TransformConfig& transform_config);

    // the transformed covariance includes the noise, the cross covariance does
    // not
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
