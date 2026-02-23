#ifndef LINE_FILTERING__LIB__LINE_TRACK_MANAGER_HPP_
#define LINE_FILTERING__LIB__LINE_TRACK_MANAGER_HPP_

#include <limits>
#include <vector>
#include <vortex_filtering/vortex_filtering.hpp>
#include "typedefs.hpp"

namespace vortex::line_filtering {

/**
 * @file line_track_manager.hpp
 * @brief Track management for 2D lines using an error-state IPDA filter.
 *
 * Each track maintains a nominal line (rho, n) and filters a 2D error
 * [delta_rho, delta_phi] via IPDA.  The "log" residual avoids angle
 * wrapping; the (rho, n) <-> (-rho, -n) ambiguity is handled by a
 * one-step sign alignment before computing residuals.
 */

/**
 * @brief Simple 2D gate operating on the [delta_rho, delta_phi] residual.
 */
struct LineGate2D {
    double min_rho_error = 0.0;
    double max_rho_error = std::numeric_limits<double>::infinity();
    double min_phi_error = 0.0;
    double max_phi_error = std::numeric_limits<double>::infinity();
    double mahalanobis_threshold = std::numeric_limits<double>::infinity();

    using Vec_z = IPDA::Vec_z;
    using Gauss_z = IPDA::Gauss_z;

    bool operator()(const Vec_z& z, const Gauss_z& z_pred) const {
        const Vec_z r = z - z_pred.mean();
        const double rho_err = std::abs(r(0));
        const double phi_err = std::abs(r(1));

        if (rho_err > max_rho_error || phi_err > max_phi_error) {
            return false;
        }

        if (rho_err <= min_rho_error && phi_err <= min_phi_error) {
            return true;
        }

        return z_pred.mahalanobis_distance(z) <= mahalanobis_threshold;
    }
};

/**
 * @brief Class responsible for maintaining a set of line tracks.
 *
 * Public API:
 *  - construct with a LineTrackManagerConfig
 *  - call step() each frame with measurements and dt
 *  - retrieve current tracks with get_tracks()
 */
class LineTrackManager {
   public:
    explicit LineTrackManager(const LineTrackManagerConfig& config);

    /**
     * @brief Perform a single tracking step.
     * @param measurements Vector of line measurements (consumed/modified)
     * @param dt Time step in seconds
     */
    void step(std::vector<LineMeasurement>& measurements, double dt);

    /**
     * @brief Get the list of currently maintained tracks.
     */
    const std::vector<LineTrack>& get_tracks() const { return tracks_; }

   private:
    /**
     * @brief Compute the 2D "log" residual between a measurement line and a
     *        track's nominal line.
     *
     * 1. Sign-align the measurement normal to the predicted normal.
     * 2. delta_rho  = rho_meas - rho_pred
     * 3. delta_phi  = atan2(cross(n_pred, n_meas), dot(n_pred, n_meas))
     *
     * @return Eigen::Vector2d [delta_rho, delta_phi]
     */
    static Eigen::Vector2d log_residual(const NominalLine& pred,
                                        LineMeasurement meas);

    /**
     * @brief "Exp" update: apply the posterior error delta to the nominal line.
     *
     * rho  <- rho + delta_rho
     * n    <- R(delta_phi) * n   (then re-normalize)
     *
     * Then reset error-state mean to zero and adjust covariance.
     */
    void inject_and_reset(LineTrack& track);

    /**
     * @brief Build the Nx2 residual array for a track against a measurement
     * set.
     */
    Eigen::Array<double, 2, Eigen::Dynamic> compute_residuals(
        const LineTrack& track,
        const std::vector<LineMeasurement>& measurements) const;

    void confirm_tracks();
    void delete_tracks();
    void create_tracks(const std::vector<LineMeasurement>& measurements);
    void sort_tracks_by_priority();

    void erase_gated_measurements(
        std::vector<LineMeasurement>& measurements,
        const Eigen::Array<bool, 1, Eigen::Dynamic>& mask) const;

    int track_id_counter_{0};
    std::vector<LineTrack> tracks_;
    LineTrackManagerConfig config_;
};

}  // namespace vortex::line_filtering

#endif  // LINE_FILTERING__LIB__LINE_TRACK_MANAGER_HPP_
