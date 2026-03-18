#ifndef LINE_FILTERING__LIB__TYPEDEFS_HPP_
#define LINE_FILTERING__LIB__TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>
#include <limits>
#include <vortex/utils/types.hpp>
#include <vortex_filtering/filters/ipda.hpp>
#include <vortex_filtering/vortex_filtering.hpp>

namespace vortex::line_filtering {

/**
 * @file typedefs.hpp
 * @brief Common type aliases and configuration structs used by the
 * line_filtering library.
 *
 * This header exposes convenient aliases for the 2D error-state line filter.
 * The error state is 2D: [delta_rho, delta_phi].
 * We use ConstantDynamicModel<2> and IdentitySensorModel<2,2> so that the
 * IPDA machinery works identically to the pose filter, but in 2D.
 */

/**
 * @brief Gaussian 2D state representation (mean + covariance).
 */
using State2d = vortex::prob::Gauss2d;

/**
 * @brief Discrete-time constant dynamic model of order 2 (delta_rho,
 * delta_phi).
 */
using DynMod = vortex::models::ConstantDynamicModel<2>;

/**
 * @brief Identity sensor model: measurement == error state.
 */
using SensorMod = vortex::models::IdentitySensorModel<2, 2>;

/**
 * @brief IPDA filter type specialized for 2D error-state line models.
 */
using IPDA = vortex::filter::IPDA<DynMod, SensorMod>;

/**
 * @brief PDAF filter type specialized for 2D error-state line models.
 */
using PDAF = vortex::filter::PDAF<DynMod, SensorMod>;

/**
 * @brief LineSegment2D type from vortex utils.
 */
using LineSegment2D = vortex::utils::types::LineSegment2D;

/**
 * @brief Parameters for existence management (track confirmation / deletion).
 */
struct ExistenceConfig {
    double confirmation_threshold{0.6};
    double deletion_threshold{0.2};
    double initial_existence_probability{0.5};
};

struct LineClassConfig {
    /// Dynamic model std dev (applied to both rho and phi error states)
    double dyn_std_dev = 0.05;
    /// Sensor model std dev
    double sens_std_dev = 0.1;

    /// Initial covariance for rho error
    double init_rho_std = 0.3;
    /// Initial covariance for phi error
    double init_phi_std = 0.15;

    /// Min rho residual below which the gate always passes (meters)
    double min_rho_error = 0.0;
    /// Max allowed rho residual for gating (meters)
    double max_rho_error = 1.0;
    /// Min phi residual below which the gate always passes (radians)
    double min_phi_error = 0.0;
    /// Max allowed phi residual for gating (radians)
    double max_phi_error = 0.5;

    /// Mahalanobis distance threshold for gating
    double mahalanobis_threshold = 3.0;

    double prob_of_detection = 0.5;
    double clutter_intensity = 0.01;
    double prob_of_survival = 0.95;
    bool estimate_clutter = false;
};

/**
 * @brief High-level track manager configuration struct.
 */
struct LineTrackManagerConfig {
    ExistenceConfig existence;
    LineClassConfig default_config;
};

/**
 * @brief Nominal state of a tracked line in (rho, n) form.
 *
 * rho   – SIGNED distance from origin to the line along the normal.
 *          Kept signed to avoid discontinuity at rho == 0.
 * n     – unit normal [nx, ny] with ||n|| == 1.
 *
 * The line equation is:  n . x = rho
 */
struct NominalLine {
    double rho{0.0};
    Eigen::Vector2d n{1.0, 0.0};
};

/**
 * @brief Representation of a single 2D line track maintained by the track
 * manager.
 */
struct LineTrack {
    /// Unique track identifier
    int id{};

    /// Nominal (rho, n) state
    NominalLine nominal;

    /// 2D error-state Gaussian [delta_rho, delta_phi]
    State2d error_state;

    /// Probability that the track exists (0..1)
    double existence_probability{0.0};

    /// Whether the track has been confirmed
    bool confirmed{false};

    /**
     * @brief Return the line in canonical form (rho >= 0, theta in [0, 2pi)).
     * This is for *display / matching only*, not used inside the filter.
     */
    vortex::utils::types::Line2D to_canonical_line() const {
        double rho_out = nominal.rho;
        double theta = std::atan2(nominal.n.y(), nominal.n.x());

        if (rho_out < 0.0) {
            rho_out = -rho_out;
            theta += M_PI;
        }
        theta = std::fmod(theta, 2.0 * M_PI);
        if (theta < 0.0) {
            theta += 2.0 * M_PI;
        }
        return vortex::utils::types::Line2D{.rho = rho_out, .theta = theta};
    }
};

/**
 * @brief A measured 2D line expressed as (rho, unit normal n).
 * Converted from LineSegment2D before entering the filter.
 */
struct LineMeasurement {
    double rho{0.0};
    Eigen::Vector2d n{1.0, 0.0};

    /**
     * @brief Create a LineMeasurement from a LineSegment2D.
     */
    static LineMeasurement from_segment(const LineSegment2D& seg) {
        const double dx = seg.p1.x - seg.p0.x;
        const double dy = seg.p1.y - seg.p0.y;
        const double len = std::hypot(dx, dy);

        if (len <= std::numeric_limits<double>::epsilon()) {
            throw std::runtime_error(
                "Invalid length for line segment in LineMeasurement");
        }

        // Normal perpendicular to the segment direction
        const double nx = -dy / len;
        const double ny = dx / len;

        // Signed rho – do NOT enforce rho >= 0
        const double rho = nx * seg.p0.x + ny * seg.p0.y;

        return LineMeasurement{.rho = rho, .n = Eigen::Vector2d{nx, ny}};
    }
};

}  // namespace vortex::line_filtering

#endif  // LINE_FILTERING__LIB__TYPEDEFS_HPP_
