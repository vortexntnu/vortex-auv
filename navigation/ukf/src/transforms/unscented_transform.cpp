#include "ukf/transforms/unscented_transform.hpp"
#include <stdexcept>

UnscentedTransform::UnscentedTransform(
    const TransformConfig& transform_config) {
    if (transform_config.alpha <= 0) {
        throw std::invalid_argument(
            "UnscentedTransform: alpha must be positive");
    }

    lambda_ = transform_config.alpha * transform_config.alpha *
                  (dimension + transform_config.kappa) -
              dimension;
    if (dimension + lambda_ <= 0) {
        throw std::invalid_argument(
            "UnscentedTransform: dimension + lambda must be positive");
    }

    // all outer points share one weight, only the centre point differs
    weights_mean_.setConstant(1.0 / (2.0 * (dimension + lambda_)));
    weights_covariance_ = weights_mean_;

    weights_mean_(0) = lambda_ / (dimension + lambda_);
    weights_covariance_(0) =
        weights_mean_(0) +
        (1 - transform_config.alpha * transform_config.alpha +
         transform_config.beta);
}
