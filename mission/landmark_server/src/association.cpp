#include "landmark_server/association.hpp"

#include <spdlog/spdlog.h>
#include <boost/math/distributions/chi_squared.hpp>

#include <algorithm>
#include <limits>
#include <optional>
#include <utility>

namespace vortex::landmark_server {

namespace {

constexpr int kDof = 3;  // bearing (2) + range (1)
constexpr std::size_t kRecentHits =
    100;  // long enough that a frame does not wander

}  // namespace

std::vector<int> solve_assignment(const Eigen::MatrixXd& cost) {
    // Shortest augmenting path with potentials, 1-indexed.
    const int n = static_cast<int>(cost.rows());
    const int m = static_cast<int>(cost.cols());
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);
    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(m + 1, inf);
        std::vector<bool> used(m + 1, false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            double delta = inf;
            int j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (used[j]) {
                    continue;
                }
                const double cur = cost(i0 - 1, j - 1) - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }
    std::vector<int> row_to_col(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            row_to_col[p[j] - 1] = j - 1;
        }
    }
    return row_to_col;
}

void Associator::reset(const Config& cfg) {
    cfg_ = &cfg;
    gate_d2_ = boost::math::quantile(boost::math::chi_squared(kDof),
                                     cfg.params.association.gate_prob);
    next_id_ = 1;
    entries_.clear();
    candidates_.clear();
    landmarks_.clear();
}

std::vector<Observation> Associator::update(const std::vector<Hit>& hits,
                                            double t,
                                            const DriftGraph& graph) {
    std::vector<Observation> out;

    std::vector<int> ids;
    for (const auto& [id, e] : entries_) {
        ids.push_back(id);
    }
    std::vector<Eigen::Vector3d> positions;
    for (const Hit& h : hits) {
        positions.push_back(h.position);
    }
    const Eigen::MatrixXd d2 = graph.squared_distances(positions, ids);

    // An object gives at most one detection per image, so one assignment per
    // image and group of classes that can be mistaken for each other.
    std::map<std::pair<double, std::string>, std::vector<Eigen::Index>> images;
    for (std::size_t i = 0; i < hits.size(); ++i) {
        images[{hits[i].t, hits[i].cls->group}].push_back(
            static_cast<Eigen::Index>(i));
    }
    const double forbidden = 1e6;
    for (const auto& [image, rows] : images) {
        std::vector<Eigen::Index> cols;
        for (std::size_t c = 0; c < ids.size(); ++c) {
            if (entries_.at(ids[c]).group == image.second) {
                cols.push_back(static_cast<Eigen::Index>(c));
            }
        }
        const auto n = static_cast<Eigen::Index>(rows.size());
        const auto m = static_cast<Eigen::Index>(cols.size());
        // One extra column per detection: no object, for the price of the gate.
        Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(n, m + n, forbidden);
        for (Eigen::Index r = 0; r < n; ++r) {
            for (Eigen::Index c = 0; c < m; ++c) {
                if (d2(rows[r], cols[c]) < gate_d2_) {
                    cost(r, c) = d2(rows[r], cols[c]);
                }
            }
            cost(r, m + r) = gate_d2_;
        }
        const std::vector<int> assignment = solve_assignment(cost);
        for (Eigen::Index r = 0; r < n; ++r) {
            const Hit& h = hits[rows[r]];
            const int c = assignment[r];
            bool in_gate = false;
            for (Eigen::Index k = 0; k < m; ++k) {
                in_gate = in_gate || d2(rows[r], cols[k]) < gate_d2_;
            }
            if (c >= m || d2(rows[r], cols[c]) >= gate_d2_) {
                // Inside a gate but taken by a better detection: not new.
                if (!in_gate) {
                    add_candidate(h);
                }
                continue;
            }
            const double best = d2(rows[r], cols[c]);
            bool ambiguous = false;
            for (Eigen::Index k = 0; k < m; ++k) {
                ambiguous = ambiguous ||
                            (k != c &&
                             d2(rows[r], cols[k]) <
                                 best + cfg_->params.association.ambiguity_d2);
            }
            if (!ambiguous) {
                record(ids[cols[c]], h, out);
            }
        }
    }
    confirm_candidates(t, graph, out);
    retire_duplicates(graph);
    rebuild_landmarks();
    return out;
}

void Associator::record(int id, const Hit& hit, std::vector<Observation>& out) {
    Entry& e = entries_.at(id);
    if (e.n_obs == 0) {
        e.first_seen = hit.t;
    }
    e.n_obs++;
    e.votes[hit.cls]++;
    e.recent.push_back(hit);
    if (e.recent.size() > kRecentHits) {
        e.recent.pop_front();
    }
    out.push_back({id, hit.position});
}

void Associator::add_candidate(const Hit& hit) {
    const AssociationParams& p = cfg_->params.association;
    Candidate* best = nullptr;
    double nearest = p.candidate_radius_m;
    for (Candidate& c : candidates_) {
        const double d = (c.position - hit.position).norm();
        if (c.group == hit.cls->group && d < nearest &&
            c.hits.back().t != hit.t) {
            best = &c;
            nearest = d;
        }
    }
    if (best == nullptr) {
        candidates_.push_back({hit.cls->group, {}, hit.position});
        best = &candidates_.back();
    }
    best->hits.push_back(hit);
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const Hit& h : best->hits) {
        sum += h.position;
    }
    best->position = sum / static_cast<double>(best->hits.size());
}

void Associator::confirm_candidates(double t,
                                    const DriftGraph& graph,
                                    std::vector<Observation>& out) {
    const AssociationParams& p = cfg_->params.association;
    const Pose T_map_odom = graph.T_map_odom();
    std::vector<Candidate> keep;
    for (Candidate& c : candidates_) {
        c.hits.erase(std::remove_if(c.hits.begin(), c.hits.end(),
                                    [&](const Hit& h) {
                                        return t - h.t > p.confirm_window_s;
                                    }),
                     c.hits.end());
        if (c.hits.empty()) {
            continue;
        }
        if (static_cast<int>(c.hits.size()) < p.confirm_hits) {
            keep.push_back(std::move(c));
            continue;
        }
        // Too close to a known object for one more: the gate was just tight.
        const Eigen::Vector3d in_map = T_map_odom * c.position;
        bool known = false;
        for (const auto& [id, e] : entries_) {
            if (e.group != c.group || e.recent.empty()) {
                continue;
            }
            const auto solved = graph.landmark(id);
            const Eigen::Vector3d at =
                solved ? *solved
                       : Eigen::Vector3d(T_map_odom * e.recent.back().position);
            known = known || (at - in_map).norm() < p.merge_radius_m;
        }
        if (known) {
            continue;
        }
        const int id = next_id_++;
        entries_[id].group = c.group;
        spdlog::info(
            "landmark_server: new landmark {} ({}) at [{:.2f}, {:.2f}, {:.2f}] "
            "from {} detections",
            id, c.hits.front().cls->name, c.position.x(), c.position.y(),
            c.position.z(), c.hits.size());
        for (const Hit& h : c.hits) {
            record(id, h, out);
        }
    }
    candidates_ = std::move(keep);
}

void Associator::retire_duplicates(const DriftGraph& graph) {
    const double radius = cfg_->params.association.merge_radius_m;
    std::vector<int> retired;
    for (auto a = entries_.begin(); a != entries_.end(); ++a) {
        for (auto b = std::next(a); b != entries_.end(); ++b) {
            if (a->second.group != b->second.group) {
                continue;
            }
            const auto pa = graph.landmark(a->first);
            const auto pb = graph.landmark(b->first);
            if (!pa || !pb || (*pa - *pb).norm() >= radius ||
                graph.squared_distance_between(a->first, b->first) >=
                    gate_d2_) {
                continue;
            }
            retired.push_back(a->second.n_obs >= b->second.n_obs ? b->first
                                                                 : a->first);
        }
    }
    for (const int id : retired) {
        spdlog::info("landmark_server: landmark {} was a duplicate", id);
        entries_.erase(id);
    }
}

void Associator::rebuild_landmarks() {
    landmarks_.clear();
    for (const auto& [id, e] : entries_) {
        if (e.recent.empty()) {
            continue;
        }
        std::vector<const Hit*> hits;
        for (const Hit& h : e.recent) {
            hits.push_back(&h);
        }
        Landmark l = summarise(hits);
        l.id = id;
        l.n_obs = e.n_obs;
        l.first_seen = e.first_seen;
        l.cls = *std::max_element(e.votes.begin(), e.votes.end(),
                                  [](const auto& a, const auto& b) {
                                      return a.second < b.second;
                                  })
                     ->first;
        landmarks_.push_back(l);
    }
}

void Associator::clear(const std::string& class_name) {
    for (auto& [id, e] : entries_) {
        const bool all = class_name.empty();
        const bool of_class =
            std::any_of(landmarks_.begin(), landmarks_.end(),
                        [&, id = id](const Landmark& l) {
                            return l.id == id && l.cls.name == class_name;
                        });
        if (all || of_class) {
            e.recent.clear();
            e.votes.clear();
            e.n_obs = 0;
        }
    }
    landmarks_.erase(std::remove_if(landmarks_.begin(), landmarks_.end(),
                                    [&](const Landmark& l) {
                                        return class_name.empty() ||
                                               l.cls.name == class_name;
                                    }),
                     landmarks_.end());
}

}  // namespace vortex::landmark_server
