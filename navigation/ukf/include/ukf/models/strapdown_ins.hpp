/**
 * @file strapdown_ins.hpp
 * @brief Contains the strapdown INS motion model of the UKF.
 */

#ifndef UKF__MODELS__STRAPDOWN_INS_HPP_
#define UKF__MODELS__STRAPDOWN_INS_HPP_

#include "ukf/typedefs.hpp"

/**
 * @brief Configuration of the StrapdownINS3D model.
 */
struct INS3DConfig {
    // continuous-time noise psd: accel, gyro, gyro-bias rw, accel-bias rw
    Eigen::Matrix12d noise_psd = Eigen::Matrix12d::Zero();
    Eigen::Vector3d gravity{0.0, 0.0, 9.82841};
};

/**
 * @brief The 3D strapdown INS motion model, driven by IMU measurements. It is
 * a Manifold with the state as point and a 15-dimensional tangent space.
 */
class StrapdownINS3D {
   public:
    static constexpr int dimension = StateIndex::size;
    using Point = State;

    /**
     * @brief Constructor for the StrapdownINS3D class.
     * @param config The noise and gravity configuration.
     */
    explicit StrapdownINS3D(const INS3DConfig& config);

    /**
     * @brief The motion model, integrating the bias corrected IMU measurement.
     * @param state The current state.
     * @param input The IMU measurement, including the time step.
     * @return The state after one time step.
     */
    State f(const State& state, const ImuInput& input) const;

    /**
     * @brief The discretised process noise covariance.
     * @param state The current state.
     * @param input The IMU measurement, including the time step.
     * @return The 15x15 process noise covariance over the time step.
     */
    Eigen::Matrix15d Q(const State& state, const ImuInput& input) const;

    /**
     * @brief Composes a state with a tangent space perturbation. The attitude
     * is composed on the right, the rest is plain addition.
     * @param state The state.
     * @param delta The perturbation of the state.
     * @return The composed state.
     */
    State composition_plus(const State& state,
                           const Eigen::Vector15d& delta) const;

    /**
     * @brief The tangent space difference between two states, the inverse of
     * composition_plus.
     * @param state_a The state we subtract from.
     * @param state_b The state we subtract.
     * @return The delta between the two states.
     */
    Eigen::Vector15d composition_minus(const State& state_a,
                                       const State& state_b) const;

   private:
    INS3DConfig config_;
};

#endif  // UKF__MODELS__STRAPDOWN_INS_HPP_
