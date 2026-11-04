#ifndef LANDMARK_SERVER__LANDMARK_HPP_
#define LANDMARK_SERVER__LANDMARK_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "landmark_server/config.hpp"

namespace vortex::landmark_server {

/// One detection, in odom.
struct Hit {
    const ClassConfig* cls{nullptr};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    std::optional<double> yaw;
    double t{0.0};
};

/// A detection that belongs to the object with this id, in odom.
struct Observation {
    int id{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

/// One object, in odom.
struct Landmark {
    int id{0};
    ClassConfig cls;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    bool yaw_known{false};
    /// Of the median: spread^2 / n.
    Eigen::Matrix3d cov{Eigen::Matrix3d::Zero()};
    double yaw_var{0.0};
    /// RMS horizontal distance of the hits from the median.
    double spread_m{0.0};
    int n_obs{0};
    double first_seen{0.0};
    double last_seen{0.0};
};

/// Densest point first: its neighbours within radius are a cluster and are
/// removed, then the next. No chaining between neighbouring objects. Only
/// clusters of at least min_size, largest first.
std::vector<std::vector<std::size_t>> cluster(
    const std::vector<Eigen::Vector3d>& points,
    double radius,
    std::size_t min_size);

/// Mean yaw of an object that looks the same after symmetry_deg (0: no
/// symmetry). nullopt without yaws.
std::optional<double> symmetric_mean_yaw(const std::vector<double>& yaws,
                                         double symmetry_deg,
                                         double* var = nullptr);

/// Median position, majority class, yaw and spread. hits must not be empty.
Landmark summarise(const std::vector<const Hit*>& hits);

/// The most observed landmark of the class, or nullptr.
const Landmark* best_of(const std::vector<Landmark>& landmarks,
                        const std::string& cls);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__LANDMARK_HPP_
