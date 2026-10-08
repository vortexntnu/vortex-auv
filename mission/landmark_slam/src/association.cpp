#include "landmark_slam/association.hpp"

#include <boost/math/distributions/chi_squared.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <set>
#include <utility>

namespace vortex::landmark_slam {

namespace {

constexpr int kDof = 3;                         // bearing (2) + range (1)
constexpr std::size_t kMaxSearchNodes = 20000;  // JCBB safety cap
// A match must be this much more likely than any other candidate of the
// detection (likelihood ratio 10: d^2 at least 2 ln 10 lower), else it is
// ambiguous and left out until the objects can be told apart.
constexpr double kAmbiguityD2 = 4.6;
constexpr double kVoteHalfLifeS = 10.0;  // a vote's weight halves
constexpr double kMinVoteWeight = 0.05;  // older votes are forgotten
constexpr std::size_t kMaxVotesPerClass = 500;
constexpr double kPriorVoteSigmas = 3.0;  // prior class: votes this many
                                          // sigma_xy from every entry are
                                          // rejected

/// One detection-landmark pairing, linearised at the current estimate.
struct Pair {
    std::size_t detection{0};
    int landmark{0};
    gtsam::Vector3 nu;
    gtsam::Matrix Hx;
    gtsam::Matrix Hl;
    gtsam::Matrix3 R;
    double d2{0.0};
};

bool same_class(const ClassConfig& a, const ClassConfig& b) {
    return (a.type == b.type && a.subtype == b.subtype) ||
           (!a.group.empty() && a.group == b.group);
}

Pair linearise(const LandmarkGraph& graph,
               int kf,
               std::size_t i,
               const Detection& d,
               int id) {
    const double range = d.z.position.norm();
    const BearingRange factor(X(kf), L(id), gtsam::Unit3(d.z.position), range,
                              gtsam::noiseModel::Unit::Create(kDof));
    Pair p;
    p.detection = i;
    p.landmark = id;
    p.nu = factor.evaluateError(graph.keyframe_pose(kf),
                                graph.landmark_pose(id), p.Hx, p.Hl);
    const gtsam::Vector3 s = bearing_range_sigmas(graph.config().params, range);
    p.R = s.cwiseProduct(s).asDiagonal();
    return p;
}

/**
 * Squared Mahalanobis distance of the stacked innovation of these pairs:
 * S = H P H^T + R over the keyframe and every landmark in the set.
 */
double joint_d2(const std::vector<const Pair*>& pairs,
                const gtsam::JointMarginal& P,
                int kf) {
    std::vector<int> ids;
    for (const Pair* p : pairs) {
        if (std::find(ids.begin(), ids.end(), p->landmark) == ids.end()) {
            ids.push_back(p->landmark);
        }
    }
    gtsam::KeyVector keys{X(kf)};
    for (int id : ids) {
        keys.push_back(L(id));
    }
    const auto m = static_cast<Eigen::Index>(kDof * pairs.size());
    const auto n = static_cast<Eigen::Index>(6 * keys.size());

    Eigen::MatrixXd Psub(n, n);
    for (std::size_t a = 0; a < keys.size(); ++a) {
        for (std::size_t b = 0; b < keys.size(); ++b) {
            Psub.block<6, 6>(6 * a, 6 * b) = P(keys[a], keys[b]);
        }
    }
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, n);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
    Eigen::VectorXd nu(m);
    for (std::size_t j = 0; j < pairs.size(); ++j) {
        const Pair& p = *pairs[j];
        const auto row = static_cast<Eigen::Index>(kDof * j);
        const auto col = static_cast<Eigen::Index>(
            6 * (1 + (std::find(ids.begin(), ids.end(), p.landmark) -
                      ids.begin())));
        H.block(row, 0, kDof, 6) = p.Hx;
        H.block(row, col, kDof, 6) = p.Hl;
        R.block(row, row, kDof, kDof) = p.R;
        nu.segment(row, kDof) = p.nu;
    }
    const Eigen::MatrixXd S = H * Psub * H.transpose() + R;
    return nu.dot(S.ldlt().solve(nu));
}

/// Depth-first branch and bound over the detections.
class Jcbb {
   public:
    Jcbb(const std::vector<std::vector<Pair>>& candidates,
         const gtsam::JointMarginal& P,
         int kf,
         double prob)
        : candidates_(candidates), P_(P), kf_(kf), prob_(prob) {}

    std::vector<const Pair*> run() {
        recurse(0);
        return best_;
    }

   private:
    void recurse(std::size_t i) {
        if (++nodes_ > kMaxSearchNodes) {
            return;
        }
        if (i == candidates_.size()) {
            if (current_.empty() || current_.size() < best_.size()) {
                return;
            }
            const double d2 = joint_d2(current_, P_, kf_);
            if (current_.size() > best_.size() || d2 < best_d2_) {
                best_ = current_;
                best_d2_ = d2;
            }
            return;
        }
        for (const Pair& p : candidates_[i]) {
            if (used_.count(p.landmark) > 0) {
                continue;  // at most one detection per landmark
            }
            current_.push_back(&p);
            const int dof = kDof * static_cast<int>(current_.size());
            if (joint_d2(current_, P_, kf_) < chi2_threshold(prob_, dof)) {
                used_.insert(p.landmark);
                recurse(i + 1);
                used_.erase(p.landmark);
            }
            current_.pop_back();
        }
        // Leave detection i unmatched if that can still tie the best.
        const std::size_t remaining = candidates_.size() - i - 1;
        if (current_.size() + remaining >= best_.size()) {
            recurse(i + 1);
        }
    }

    const std::vector<std::vector<Pair>>& candidates_;
    const gtsam::JointMarginal& P_;
    int kf_;
    double prob_;
    std::vector<const Pair*> current_;
    std::vector<const Pair*> best_;
    double best_d2_{std::numeric_limits<double>::infinity()};
    std::set<int> used_;
    std::size_t nodes_{0};
};

double vote_weight(const Vote& v, double t) {
    return std::exp2(-std::max(0.0, t - v.t) / kVoteHalfLifeS);
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
    const double value = boost::math::quantile(dist, prob);
    cache[key] = value;
    return value;
}

Association associate(const LandmarkGraph& graph,
                      int kf,
                      const std::vector<Detection>& detections) {
    Association out;
    const double prob = graph.config().params.gate_prob;

    // Joint marginal over the keyframe and every same-class landmark.
    gtsam::KeyVector keys{X(kf)};
    for (const LandmarkState& l : graph.landmarks()) {
        for (const Detection& d : detections) {
            if (same_class(*d.cls, l.cls)) {
                keys.push_back(L(l.id));
                break;
            }
        }
    }
    if (keys.size() == 1) {
        for (std::size_t i = 0; i < detections.size(); ++i) {
            out.unmatched.push_back(i);
        }
        return out;
    }
    const gtsam::JointMarginal P = graph.joint_cov(keys);

    // Individual compatibility.
    std::vector<std::vector<Pair>> candidates(detections.size());
    for (std::size_t i = 0; i < detections.size(); ++i) {
        for (const LandmarkState& l : graph.landmarks()) {
            if (!same_class(*detections[i].cls, l.cls)) {
                continue;
            }
            Pair p = linearise(graph, kf, i, detections[i], l.id);
            p.d2 = joint_d2({&p}, P, kf);
            if (p.d2 < chi2_threshold(prob, kDof)) {
                candidates[i].push_back(p);
            }
        }
        std::sort(candidates[i].begin(), candidates[i].end(),
                  [](const Pair& a, const Pair& b) { return a.d2 < b.d2; });
    }

    const std::vector<const Pair*> best = Jcbb(candidates, P, kf, prob).run();
    std::vector<bool> matched(detections.size(), false);
    for (const Pair* p : best) {
        matched[p->detection] = true;
        const bool ambiguous = std::any_of(
            candidates[p->detection].begin(), candidates[p->detection].end(),
            [&](const Pair& q) {
                return q.landmark != p->landmark && q.d2 < p->d2 + kAmbiguityD2;
            });
        if (!ambiguous) {
            out.matches.push_back({p->detection, p->landmark, p->d2 / kDof});
        }
    }
    // Only a detection no landmark could explain may start a new one: an
    // unmatched detection with a compatible landmark is a second look at it
    // (or clutter next to it).
    for (std::size_t i = 0; i < detections.size(); ++i) {
        if (!matched[i] && candidates[i].empty()) {
            out.unmatched.push_back(i);
        }
    }
    return out;
}

void Votes::add(const Detection& d,
                int kf,
                const gtsam::Pose3& T_map_base,
                double t,
                const Config& cfg) {
    Vote v;
    v.kf = kf;
    v.z = d.z;
    v.point = T_map_base.transformFrom(d.z.position);
    v.t = t;
    if (d.z.rotation && d.cls->has_orientation) {
        v.yaw = (T_map_base.rotation() * *d.z.rotation).yaw();
    }

    // A class in the prior map: only votes near one of its entries count.
    bool in_prior = false;
    bool near_prior = false;
    for (const PriorLandmark& pl : cfg.prior_landmarks) {
        const auto pl_cls = std::find_if(
            cfg.classes.begin(), cfg.classes.end(),
            [&](const ClassConfig& c) { return c.name == pl.class_name; });
        if (pl_cls == cfg.classes.end() || !same_class(*pl_cls, *d.cls)) {
            continue;
        }
        in_prior = true;
        const double dxy = std::hypot(v.point.x() - pl.x, v.point.y() - pl.y);
        near_prior = near_prior || dxy <= kPriorVoteSigmas * pl.sigma_xy +
                                              cfg.params.vote_radius_m;
    }
    if (in_prior && !near_prior) {
        return;
    }

    auto it = std::find_if(
        classes_.begin(), classes_.end(),
        [&](const ClassVotes& c) { return same_class(c.cls, *d.cls); });
    if (it == classes_.end()) {
        classes_.push_back({*d.cls, {}});
        it = std::prev(classes_.end());
    }
    it->votes.push_back(v);
    if (it->votes.size() > kMaxVotesPerClass) {
        it->votes.erase(it->votes.begin());
    }
}

std::vector<VoteCluster> Votes::take_clusters(const LandmarkGraph& graph,
                                              double t) {
    const Params& p = graph.config().params;
    const double r = p.vote_radius_m;
    std::vector<VoteCluster> out;

    for (ClassVotes& cv : classes_) {
        auto& votes = cv.votes;
        votes.erase(std::remove_if(votes.begin(), votes.end(),
                                   [&](const Vote& v) {
                                       return vote_weight(v, t) <
                                              kMinVoteWeight;
                                   }),
                    votes.end());

        while (!votes.empty()) {
            // Seed: the vote with the most vote weight around it.
            std::size_t seed = 0;
            double seed_weight = 0.0;
            for (std::size_t i = 0; i < votes.size(); ++i) {
                double w = 0.0;
                for (const Vote& v : votes) {
                    if ((v.point - votes[i].point).norm() < r) {
                        w += vote_weight(v, t);
                    }
                }
                if (w > seed_weight) {
                    seed = i;
                    seed_weight = w;
                }
            }
            if (seed_weight < p.min_votes) {
                break;
            }

            // Weighted centroid of the votes around the seed, then the
            // votes around that centroid.
            const auto centroid = [&](const gtsam::Point3& c) {
                gtsam::Point3 sum = gtsam::Point3::Zero();
                double w_sum = 0.0;
                for (const Vote& v : votes) {
                    if ((v.point - c).norm() < r) {
                        sum += vote_weight(v, t) * v.point;
                        w_sum += vote_weight(v, t);
                    }
                }
                return gtsam::Point3(sum / w_sum);
            };
            VoteCluster cluster;
            cluster.position = centroid(centroid(votes[seed].point));
            std::vector<Vote> rest;
            for (Vote& v : votes) {
                if ((v.point - cluster.position).norm() < r) {
                    if (v.yaw) {
                        cluster.yaw = v.yaw;
                    }
                    cluster.votes.push_back(std::move(v));
                } else {
                    rest.push_back(std::move(v));
                }
            }
            votes = std::move(rest);
            if (cluster.votes.empty()) {
                break;
            }
            // Its class: the one most of its votes gave (a class group).
            std::map<std::string, std::pair<const ClassConfig*, int>> count;
            for (const Vote& v : cluster.votes) {
                ++count.try_emplace(v.z.cls->name, v.z.cls, 0)
                      .first->second.second;
            }
            cluster.cls =
                *std::max_element(count.begin(), count.end(),
                                  [](const auto& a, const auto& b) {
                                      return a.second.second < b.second.second;
                                  })
                     ->second.first;

            // At a landmark of the class already: a second look at it.
            const bool known = std::any_of(
                graph.landmarks().begin(), graph.landmarks().end(),
                [&](const LandmarkState& l) {
                    return same_class(l.cls, cv.cls) &&
                           (l.pose.translation() - cluster.position).norm() < r;
                });
            if (!known) {
                out.push_back(std::move(cluster));
            }
        }
    }
    return out;
}

}  // namespace vortex::landmark_slam
