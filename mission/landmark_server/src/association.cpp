#include "landmark_server/association.hpp"

#include <boost/math/distributions/chi_squared.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace vortex::landmark_server {

namespace {

constexpr int kDof = 3;  // bearing (2) + range (1)
// A candidate keeps at most this many hits (the newest): enough to start a
// landmark with, bounded while the candidate waits.
constexpr std::size_t kMaxHitsPerCandidate = 50;

/// Squared Mahalanobis distance of a detection to a landmark, linearised at
/// the current estimate: S = H P H^T + R over the keyframe and the landmark.
double pair_d2(const LandmarkGraph& graph,
               int kf,
               const Measurement& z,
               int id,
               const gtsam::JointMarginal& P) {
    const double range = z.position.norm();
    const BearingRange factor(X(kf), L(id), gtsam::Unit3(z.position), range,
                              gtsam::noiseModel::Unit::Create(kDof));
    gtsam::Matrix Hx;
    gtsam::Matrix Hl;
    const gtsam::Vector nu = factor.evaluateError(
        graph.keyframe_pose(kf), graph.landmark_pose(id), Hx, Hl);
    Eigen::Matrix<double, kDof, 12> H;
    H << Hx, Hl;
    Eigen::Matrix<double, 12, 12> Pxl;
    Pxl << P(X(kf), X(kf)), P(X(kf), L(id)), P(L(id), X(kf)), P(L(id), L(id));
    const gtsam::Vector3 s = bearing_range_sigmas(graph.params(), range);
    const Eigen::Matrix3d S =
        H * Pxl * H.transpose() + Eigen::Matrix3d(s.cwiseAbs2().asDiagonal());
    return nu.dot(S.ldlt().solve(nu));
}

}  // namespace

double chi2_threshold(double prob, int dof) {
    static std::map<std::pair<double, int>, double> cache;
    const auto key = std::make_pair(prob, dof);
    const auto it = cache.find(key);
    if (it != cache.end()) {
        return it->second;
    }
    const boost::math::chi_squared dist(dof);
    return cache[key] = boost::math::quantile(dist, prob);
}

std::vector<int> solve_assignment(const Eigen::MatrixXd& cost) {
    // Shortest augmenting path with potentials, 1-indexed (row/col 0 is a
    // virtual start).
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

Association associate(const LandmarkGraph& graph,
                      int kf,
                      const std::vector<Detection>& detections) {
    Association out;
    const Params& params = graph.params();
    const double gate = chi2_threshold(params.gate_prob, kDof);

    // The landmarks of the classes in this message.
    std::vector<const LandmarkState*> landmarks;
    for (const LandmarkState& l : graph.landmarks()) {
        if (std::any_of(detections.begin(), detections.end(),
                        [&](const Detection& d) {
                            return d.cls->name == l.cls.name;
                        })) {
            landmarks.push_back(&l);
        }
    }
    if (landmarks.empty()) {
        for (std::size_t i = 0; i < detections.size(); ++i) {
            out.unmatched.push_back(i);
        }
        return out;
    }
    gtsam::KeyVector keys{X(kf)};
    for (const LandmarkState* l : landmarks) {
        keys.push_back(L(l->id));
    }
    const gtsam::JointMarginal P = graph.joint_cov(keys);

    // Per class: detections x (landmarks + one "unpaired" column per
    // detection, costing the gate). Forbidden pairs cost more than any
    // solution with them unpaired.
    std::vector<std::string> classes;
    for (const Detection& d : detections) {
        if (std::find(classes.begin(), classes.end(), d.cls->name) ==
            classes.end()) {
            classes.push_back(d.cls->name);
        }
    }
    for (const std::string& cls : classes) {
        std::vector<std::size_t> dets;
        for (std::size_t i = 0; i < detections.size(); ++i) {
            if (detections[i].cls->name == cls) {
                dets.push_back(i);
            }
        }
        std::vector<const LandmarkState*> lms;
        for (const LandmarkState* l : landmarks) {
            if (l->cls.name == cls) {
                lms.push_back(l);
            }
        }
        const auto n = static_cast<Eigen::Index>(dets.size());
        const auto m = static_cast<Eigen::Index>(lms.size());
        Eigen::MatrixXd d2 = Eigen::MatrixXd::Constant(
            n, m, std::numeric_limits<double>::infinity());
        for (Eigen::Index r = 0; r < n; ++r) {
            for (Eigen::Index c = 0; c < m; ++c) {
                d2(r, c) =
                    pair_d2(graph, kf, detections[dets[r]].z, lms[c]->id, P);
            }
        }
        const double forbidden = 1e6 + 2.0 * static_cast<double>(n) * gate;
        Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(n, m + n, forbidden);
        for (Eigen::Index r = 0; r < n; ++r) {
            for (Eigen::Index c = 0; c < m; ++c) {
                if (d2(r, c) < gate) {
                    cost(r, c) = d2(r, c);
                }
            }
            cost(r, m + r) = gate;
        }
        const std::vector<int> assignment = solve_assignment(cost);
        for (Eigen::Index r = 0; r < n; ++r) {
            const int c = assignment[r];
            const bool in_gate = (d2.row(r).array() < gate).any();
            if (c < 0 || c >= m || d2(r, c) >= gate) {
                if (!in_gate) {
                    out.unmatched.push_back(dets[r]);
                }
                continue;
            }
            // Ambiguous: another landmark explains it nearly as well.
            bool ambiguous = false;
            for (Eigen::Index k = 0; k < m; ++k) {
                ambiguous =
                    ambiguous ||
                    (k != c && d2(r, k) < d2(r, c) + params.ambiguity_d2);
            }
            if (!ambiguous) {
                out.matches.push_back({dets[r], lms[c]->id, d2(r, c) / kDof});
            }
        }
    }
    return out;
}

void Candidates::add(const std::vector<Detection>& detections,
                     const std::vector<std::size_t>& unmatched,
                     int kf,
                     const gtsam::Pose3& T_map_base,
                     double t,
                     const Params& params,
                     const std::map<std::string, gtsam::Point3>& priors) {
    // Candidates hit by this message: one detection each.
    std::vector<std::size_t> hit_now;
    for (const std::size_t i : unmatched) {
        const Detection& d = detections[i];
        Hit h;
        h.kf = kf;
        h.z = d.z;
        h.point = T_map_base.transformFrom(d.z.position);
        h.t = t;
        if (d.z.rotation && d.cls->has_orientation) {
            h.yaw = (T_map_base.rotation() * *d.z.rotation).yaw();
        }
        // Far from where the class's task is: a false detection.
        if (!d.cls->prior.empty()) {
            const auto it = priors.find(d.cls->prior);
            const double dist = it == priors.end()
                                    ? 0.0
                                    : std::hypot(h.point.x() - it->second.x(),
                                                 h.point.y() - it->second.y());
            if (dist > d.cls->prior_radius_m) {
                auto& [count, nearest] =
                    prior_rejects_.try_emplace(d.cls->name, 0, dist)
                        .first->second;
                ++count;
                nearest = std::min(nearest, dist);
                continue;
            }
        }
        std::optional<std::size_t> best;
        double best_dist = params.candidate_radius_m;
        for (std::size_t k = 0; k < candidates_.size(); ++k) {
            const Candidate& c = candidates_[k];
            const double dist = (c.position - h.point).norm();
            if (c.cls->name == d.cls->name && dist < best_dist &&
                std::find(hit_now.begin(), hit_now.end(), k) == hit_now.end()) {
                best = k;
                best_dist = dist;
            }
        }
        if (!best) {
            candidates_.push_back({d.cls, h.point, {}});
            best = candidates_.size() - 1;
        }
        Candidate& c = candidates_[*best];
        c.hits.push_back(h);
        if (c.hits.size() > kMaxHitsPerCandidate) {
            c.hits.erase(c.hits.begin());
        }
        gtsam::Point3 sum = gtsam::Point3::Zero();
        for (const Hit& k : c.hits) {
            sum += k.point;
        }
        c.position = sum / static_cast<double>(c.hits.size());
        hit_now.push_back(*best);
    }
}

std::vector<Candidate> Candidates::take_confirmed(const LandmarkGraph& graph,
                                                  double t) {
    const Params& p = graph.params();
    std::vector<Candidate> out;
    std::vector<Candidate> keep;
    for (Candidate& c : candidates_) {
        if (c.hits.empty() || t - c.hits.back().t > p.confirm_window_s) {
            continue;  // forgotten
        }
        const auto recent = std::count_if(
            c.hits.begin(), c.hits.end(),
            [&](const Hit& h) { return t - h.t <= p.confirm_window_s; });
        if (recent < p.confirm_hits) {
            keep.push_back(std::move(c));
            continue;
        }
        const bool known =
            std::any_of(graph.landmarks().begin(), graph.landmarks().end(),
                        [&](const LandmarkState& l) {
                            return l.cls.name == c.cls->name &&
                                   (l.pose.translation() - c.position).norm() <
                                       p.merge_radius_m;
                        });
        if (!known) {
            out.push_back(std::move(c));
        }
    }
    candidates_ = std::move(keep);
    return out;
}

}  // namespace vortex::landmark_server
