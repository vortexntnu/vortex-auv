#include "landmark_server/measure.hpp"

#include <fmt/format.h>

#include <cmath>
#include <utility>

namespace vortex::landmark_server {

namespace {

std::vector<Landmark> clusters_of(const std::vector<const Hit*>& hits,
                                  const MeasureSettings& s) {
    std::vector<Eigen::Vector3d> points;
    for (const Hit* h : hits) {
        points.push_back(h->position);
    }
    std::vector<Landmark> out;
    for (const auto& members : cluster(points, s.cluster_radius_m,
                                       static_cast<std::size_t>(s.min_hits))) {
        std::vector<const Hit*> in;
        for (const std::size_t i : members) {
            in.push_back(hits[i]);
        }
        out.push_back(summarise(in));
    }
    return out;
}

double yaw_from_vehicle(const Pose& T_odom_base, const Eigen::Vector3d& p) {
    const Eigen::Vector3d d = p - T_odom_base.translation();
    return std::atan2(d.y(), d.x());
}

Measurement failed(std::string message, int hits) {
    Measurement m;
    m.hits = hits;
    m.message = std::move(message);
    return m;
}

}  // namespace

Measurement measure_object(const std::vector<Hit>& hits,
                           const Pose& T_odom_base,
                           const MeasureSettings& s) {
    std::vector<const Hit*> all;
    for (const Hit& h : hits) {
        all.push_back(&h);
    }
    const int n = static_cast<int>(hits.size());
    const auto clusters = clusters_of(all, s);
    if (clusters.empty()) {
        return failed(fmt::format("{} hits, no {} within {:.2f} m", n,
                                  s.min_hits, s.cluster_radius_m),
                      n);
    }
    const Landmark& l = clusters.front();
    Measurement m;
    m.hits = l.n_obs;
    m.spread_m = l.spread_m;
    if (l.spread_m > s.max_spread_m) {
        m.message = fmt::format("spread {:.2f} m > {:.2f} m", l.spread_m,
                                s.max_spread_m);
        return m;
    }
    const double yaw =
        l.yaw_known ? l.yaw : yaw_from_vehicle(T_odom_base, l.position);
    m.pose = yaw_pose(yaw, l.position);
    m.ok = true;
    m.message = fmt::format("{} hits, spread {:.2f} m", m.hits, m.spread_m);
    return m;
}

}  // namespace vortex::landmark_server
