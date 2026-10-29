#include "landmark_server/map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace vortex::landmark_server {

namespace {

constexpr double kMatchRadiusFactor = 2.0;

double median(std::vector<double> v) {
    const std::size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    double m = v[mid];
    if (v.size() % 2 == 0) {
        m = 0.5 * (m + *std::max_element(v.begin(), v.begin() + mid));
    }
    return m;
}

}  // namespace

std::vector<std::vector<std::size_t>> cluster(
    const std::vector<Eigen::Vector3d>& points,
    double radius,
    std::size_t min_size) {
    const double r2 = radius * radius;
    std::vector<bool> alive(points.size(), true);
    std::vector<std::vector<std::size_t>> out;
    while (true) {
        std::size_t best = 0;
        std::size_t best_count = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (!alive[i]) {
                continue;
            }
            std::size_t count = 0;
            for (std::size_t j = 0; j < points.size(); ++j) {
                count +=
                    alive[j] && (points[i] - points[j]).squaredNorm() <= r2;
            }
            if (count > best_count) {
                best = i;
                best_count = count;
            }
        }
        if (best_count < min_size || best_count == 0) {
            return out;
        }
        std::vector<std::size_t> members;
        for (std::size_t j = 0; j < points.size(); ++j) {
            if (alive[j] && (points[best] - points[j]).squaredNorm() <= r2) {
                members.push_back(j);
                alive[j] = false;
            }
        }
        out.push_back(std::move(members));
    }
}

std::optional<double> symmetric_mean_yaw(const std::vector<double>& yaws,
                                         double symmetry_deg,
                                         double* var) {
    if (yaws.empty() || symmetry_deg >= 360.0) {
        return std::nullopt;
    }
    // Yaw modulo the symmetry: average k * yaw on the circle, then divide.
    const double k = symmetry_deg > 0.0 ? 360.0 / symmetry_deg : 1.0;
    double s = 0.0;
    double c = 0.0;
    for (const double y : yaws) {
        s += std::sin(k * y);
        c += std::cos(k * y);
    }
    const double n = static_cast<double>(yaws.size());
    if (var) {
        const double r = std::clamp(std::hypot(s, c) / n, 1e-9, 1.0);
        *var = -2.0 * std::log(r) / (k * k) / n;
    }
    return std::atan2(s, c) / k;
}

Landmark summarise(const std::vector<const Hit*>& hits) {
    std::map<const ClassConfig*, int> votes;
    for (const Hit* h : hits) {
        votes[h->cls]++;
    }
    const ClassConfig* cls =
        std::max_element(votes.begin(), votes.end(), [](auto& a, auto& b) {
            return a.second < b.second;
        })->first;

    Landmark l;
    l.cls = *cls;
    l.n_obs = static_cast<int>(hits.size());
    l.first_seen = std::numeric_limits<double>::max();
    std::vector<double> axis[3];
    std::vector<double> yaws;
    for (const Hit* h : hits) {
        for (int i = 0; i < 3; ++i) {
            axis[i].push_back(h->position[i]);
        }
        if (h->yaw) {
            yaws.push_back(*h->yaw);
        }
        l.first_seen = std::min(l.first_seen, h->t);
        l.last_seen = std::max(l.last_seen, h->t);
    }
    for (int i = 0; i < 3; ++i) {
        l.position[i] = median(axis[i]);
    }
    for (const Hit* h : hits) {
        const Eigen::Vector3d d = h->position - l.position;
        l.cov += d * d.transpose();
        l.spread_m += d.head<2>().squaredNorm();
    }
    const double n = static_cast<double>(hits.size());
    l.spread_m = std::sqrt(l.spread_m / n);
    l.cov /= n * std::max(n - 1.0, 1.0);
    if (cls->has_orientation) {
        if (const auto yaw =
                symmetric_mean_yaw(yaws, cls->symmetry_deg, &l.yaw_var)) {
            l.yaw = *yaw;
            l.yaw_known = true;
        }
    }
    return l;
}

void DetectionMap::add(const Hit& hit, std::size_t max_hits) {
    auto& q = hits_[hit.cls->name];
    q.push_back(hit);
    while (q.size() > max_hits) {
        q.pop_front();
    }
}

void DetectionMap::clear() {
    hits_.clear();
    landmarks_.clear();
}

void DetectionMap::clear_class(const std::string& name) {
    hits_.erase(name);
    landmarks_.erase(
        std::remove_if(landmarks_.begin(), landmarks_.end(),
                       [&](const Landmark& l) { return l.cls.name == name; }),
        landmarks_.end());
}

void DetectionMap::update(double cluster_radius_m,
                          int confirm_hits,
                          double max_age_s,
                          double t) {
    std::map<std::string, std::vector<const Hit*>> groups;
    for (auto& [name, q] : hits_) {
        while (max_age_s > 0.0 && !q.empty() && t - q.front().t > max_age_s) {
            q.pop_front();
        }
        for (const Hit& h : q) {
            groups[h.cls->group].push_back(&h);
        }
    }

    std::vector<Landmark> next;
    for (const auto& [group, hits] : groups) {
        std::vector<Eigen::Vector3d> points;
        for (const Hit* h : hits) {
            points.push_back(h->position);
        }
        for (const auto& members :
             cluster(points, cluster_radius_m,
                     static_cast<std::size_t>(confirm_hits))) {
            std::vector<const Hit*> in;
            for (const std::size_t i : members) {
                in.push_back(hits[i]);
            }
            next.push_back(summarise(in));
        }
    }

    // Stable ids: nearest landmark of the last round in the same group.
    std::vector<std::tuple<double, std::size_t, std::size_t>> pairs;
    for (std::size_t i = 0; i < next.size(); ++i) {
        for (std::size_t j = 0; j < landmarks_.size(); ++j) {
            const double d = (next[i].position - landmarks_[j].position).norm();
            if (next[i].cls.group == landmarks_[j].cls.group &&
                d < kMatchRadiusFactor * cluster_radius_m) {
                pairs.emplace_back(d, i, j);
            }
        }
    }
    std::sort(pairs.begin(), pairs.end());
    std::vector<bool> taken_new(next.size(), false);
    std::vector<bool> taken_old(landmarks_.size(), false);
    for (const auto& [d, i, j] : pairs) {
        if (!taken_new[i] && !taken_old[j]) {
            taken_new[i] = taken_old[j] = true;
            next[i].id = landmarks_[j].id;
            next[i].first_seen =
                std::min(next[i].first_seen, landmarks_[j].first_seen);
        }
    }
    for (std::size_t i = 0; i < next.size(); ++i) {
        if (!taken_new[i]) {
            next[i].id = next_id_++;
        }
    }
    landmarks_ = std::move(next);
}

}  // namespace vortex::landmark_server
