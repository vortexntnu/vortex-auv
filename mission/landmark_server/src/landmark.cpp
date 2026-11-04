#include "landmark_server/landmark.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vortex::landmark_server {

namespace {

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
    // Neighbours are found once; taking out a cluster only lowers the counts
    // of what was next to it. Same result as recounting every round.
    const double r2 = radius * radius;
    const std::size_t n = points.size();
    std::vector<std::vector<std::size_t>> near(n);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            if ((points[i] - points[j]).squaredNorm() <= r2) {
                near[i].push_back(j);
                near[j].push_back(i);
            }
        }
    }
    std::vector<std::size_t> count(n);
    for (std::size_t i = 0; i < n; ++i) {
        count[i] = near[i].size() + 1;
    }
    std::vector<bool> alive(n, true);
    std::vector<std::vector<std::size_t>> out;
    while (true) {
        std::size_t best = 0;
        std::size_t best_count = 0;
        for (std::size_t i = 0; i < n; ++i) {
            if (alive[i] && count[i] > best_count) {
                best = i;
                best_count = count[i];
            }
        }
        if (best_count < min_size || best_count == 0) {
            return out;
        }
        std::vector<std::size_t> members{best};
        for (const std::size_t j : near[best]) {
            if (alive[j]) {
                members.push_back(j);
            }
        }
        std::sort(members.begin(), members.end());
        for (const std::size_t m : members) {
            alive[m] = false;
        }
        for (const std::size_t m : members) {
            for (const std::size_t k : near[m]) {
                if (alive[k]) {
                    --count[k];
                }
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

const Landmark* best_of(const std::vector<Landmark>& landmarks,
                        const std::string& cls) {
    const Landmark* best = nullptr;
    for (const auto& l : landmarks) {
        if (l.cls.name == cls && (!best || l.n_obs > best->n_obs)) {
            best = &l;
        }
    }
    return best;
}

}  // namespace vortex::landmark_server
