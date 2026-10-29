#ifndef LANDMARK_SERVER__MAP_HPP_
#define LANDMARK_SERVER__MAP_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <deque>
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

/// Stored detections per class, clustered again per class group on every
/// update.
class DetectionMap {
   public:
    /// Keeps the newest max_hits of the class.
    void add(const Hit& hit, std::size_t max_hits);
    void clear();
    /// Drops the hits and landmarks of one class.
    void clear_class(const std::string& name);
    /// max_age_s > 0 forgets hits older than that before t.
    void update(double cluster_radius_m,
                int confirm_hits,
                double max_age_s,
                double t);
    const std::vector<Landmark>& landmarks() const { return landmarks_; }

   private:
    std::map<std::string, std::deque<Hit>> hits_;  // per class
    std::vector<Landmark> landmarks_;
    int next_id_{1};
};

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__MAP_HPP_
