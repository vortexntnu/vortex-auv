#ifndef UKF__MODELS__STRAPDOWN_INS_HPP_
#define UKF__MODELS__STRAPDOWN_INS_HPP_

#include "ukf/typedefs.hpp"

struct INS3DConfig {
    // continuous-time noise psd: accel, gyro, gyro-bias rw, accel-bias rw
    Eigen::Matrix12d noise_psd = Eigen::Matrix12d::Zero();
    Eigen::Vector3d gravity{0.0, 0.0, 9.82841};
};

class StrapdownINS3D {
   public:
    static constexpr int dimension = StateIndex::size;
    using Point = State;

    explicit StrapdownINS3D(const INS3DConfig& config);

    State f(const State& state, const ImuInput& input) const;

    Eigen::Matrix15d Q(const State& state, const ImuInput& input) const;

    State composition_plus(const State& state,
                           const Eigen::Vector15d& delta) const;

    Eigen::Vector15d composition_minus(const State& state_a,
                                       const State& state_b) const;

   private:
    INS3DConfig config_;
};

#endif  // UKF__MODELS__STRAPDOWN_INS_HPP_
