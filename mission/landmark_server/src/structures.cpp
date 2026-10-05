#include "landmark_server/structures.hpp"
#include <algorithm>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <tuple>
#include "landmark_server/class_config.hpp"

namespace vortex::mission {

namespace {

/// Two members closer than this horizontally give no yaw [m].
constexpr double kMinBaseline = 0.15;
constexpr int kRefineIterations = 6;

Eigen::Matrix3d yaw_rotation(double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Isometry3d make_pose(double yaw, const Eigen::Vector3d& t) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = yaw_rotation(yaw);
    T.translation() = t;
    return T;
}

double yaw_of(const Eigen::Isometry3d& T) {
    return std::atan2(T.linear()(1, 0), T.linear()(0, 0));
}

Eigen::Vector3d parse_vec3(const YAML::Node& node, const std::string& where) {
    if (!node.IsSequence() || node.size() != 3) {
        throw std::runtime_error(where + " must be [x, y, z]");
    }
    return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

/// A key that is not in @p allowed is a typo: an error, not a silent default.
void check_keys(const YAML::Node& node,
                std::initializer_list<const char*> allowed,
                const std::string& where) {
    if (!node.IsMap()) {
        return;
    }
    for (const auto& kv : node) {
        const auto key = kv.first.as<std::string>();
        if (std::none_of(allowed.begin(), allowed.end(),
                         [&](const char* a) { return key == a; })) {
            throw std::runtime_error(where + ": unknown key '" + key + "'");
        }
    }
}

LandmarkClassKey parse_key(const std::string& name, const std::string& where) {
    const auto parsed = parse_class_name(name);
    if (!parsed) {
        throw std::runtime_error(where + ": unknown class '" + name + "'");
    }
    return {parsed->first, parsed->second};
}

std::vector<DerivedPoint> parse_points(const YAML::Node& node,
                                       const std::vector<StructureMember>& members,
                                       const std::string& where) {
    std::vector<DerivedPoint> out;
    if (!node) {
        return out;
    }
    for (const auto& kv : node) {
        DerivedPoint p;
        p.name = kv.first.as<std::string>();
        const std::string w = where + "." + p.name;
        check_keys(kv.second, {"class", "offset", "from", "yaw_deg"}, w);
        if (!kv.second["class"]) {
            throw std::runtime_error(w + ": missing class");
        }
        p.cls = parse_key(kv.second["class"].as<std::string>(), w);
        if (kv.second["offset"]) {
            p.offset = parse_vec3(kv.second["offset"], w + ".offset");
        }
        if (kv.second["from"]) {
            p.from = kv.second["from"].as<std::string>();
            const bool known = std::any_of(members.begin(), members.end(),
                                           [&](const auto& m) { return m.name == p.from; });
            if (!known) {
                throw std::runtime_error(w + ".from: no part '" + p.from + "'");
            }
        }
        if (kv.second["yaw_deg"]) {
            p.yaw = kv.second["yaw_deg"].as<double>() * M_PI / 180.0;
        }
        out.push_back(std::move(p));
    }
    return out;
}

std::vector<StructureMember> parse_members(const YAML::Node& node,
                                           const Eigen::Vector3d& sigma,
                                           const std::string& where) {
    if (!node || !node.IsMap() || node.size() == 0) {
        throw std::runtime_error(where + ": a structure needs members");
    }
    std::vector<StructureMember> out;
    for (const auto& kv : node) {
        StructureMember m;
        m.name = kv.first.as<std::string>();
        const std::string w = where + "." + m.name;
        check_keys(kv.second, {"class", "offset", "sigma"}, w);
        const auto cls = kv.second["class"];
        if (!cls) {
            throw std::runtime_error(w + ": missing class");
        }
        if (cls.IsSequence()) {
            for (const auto& c : cls) {
                m.classes.push_back(parse_key(c.as<std::string>(), w));
            }
        } else {
            m.classes.push_back(parse_key(cls.as<std::string>(), w));
        }
        if (kv.second["offset"]) {
            m.offset = parse_vec3(kv.second["offset"], w + ".offset");
        }
        m.sigma = kv.second["sigma"] ? parse_vec3(kv.second["sigma"], w + ".sigma")
                                     : sigma;
        if ((m.sigma.array() <= 0.0).any()) {
            throw std::runtime_error(w + ".sigma must be > 0");
        }
        out.push_back(std::move(m));
    }
    return out;
}

Eigen::Matrix3d member_cov(const StructureMember& m,
                           const Eigen::Matrix3d& R,
                           const FitLandmark& lm) {
    return R * m.sigma.cwiseAbs2().asDiagonal() * R.transpose() +
           lm.covariance;
}

struct Hypothesis {
    double yaw{0.0};
    Eigen::Vector3d t{Eigen::Vector3d::Zero()};
    std::vector<std::pair<std::size_t, std::size_t>> members;  // (member, lm)
    double cost{0.0};
};

void assign(const StructureTemplate& tmpl,
            const StructureVariant& v,
            const std::vector<FitLandmark>& lms,
            Hypothesis& h) {
    const Eigen::Isometry3d pose = make_pose(h.yaw, h.t);
    std::vector<std::tuple<double, std::size_t, std::size_t>> pairs;
    for (std::size_t s = 0; s < v.members.size(); ++s) {
        for (std::size_t l = 0; l < lms.size(); ++l) {
            if (!v.members[s].accepts(lms[l].key)) {
                continue;
            }
            const double c = member_chi2(v.members[s], pose, lms[l]);
            if (c <= tmpl.member_gate_chi2) {
                pairs.emplace_back(c, s, l);
            }
        }
    }
    std::sort(pairs.begin(), pairs.end());
    std::vector<bool> member_used(v.members.size(), false);
    std::vector<bool> lm_used(lms.size(), false);
    h.members.clear();
    h.cost = 0.0;
    for (const auto& [c, s, l] : pairs) {
        if (member_used[s] || lm_used[l]) {
            continue;
        }
        member_used[s] = lm_used[l] = true;
        h.members.emplace_back(s, l);
        h.cost += c;
    }
}

/// Weighted least squares on (yaw, t) with the assigned members.
void refine(const StructureVariant& v,
            const std::vector<FitLandmark>& lms,
            Hypothesis& h) {
    for (int it = 0; it < kRefineIterations && !h.members.empty(); ++it) {
        const Eigen::Matrix3d R = yaw_rotation(h.yaw);
        const double c = std::cos(h.yaw);
        const double s = std::sin(h.yaw);
        Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
        Eigen::Vector4d b = Eigen::Vector4d::Zero();
        for (const auto& [mi, li] : h.members) {
            const auto& m = v.members[mi];
            const auto& lm = lms[li];
            const Eigen::Vector3d& o = m.offset;
            const Eigen::Vector3d r = lm.position - (h.t + R * o);
            Eigen::Matrix<double, 3, 4> J = Eigen::Matrix<double, 3, 4>::Zero();
            J(0, 0) = -s * o.x() - c * o.y();
            J(1, 0) = c * o.x() - s * o.y();
            J.rightCols<3>().setIdentity();
            const Eigen::Matrix3d W = member_cov(m, R, lm).inverse();
            A += J.transpose() * W * J;
            b += J.transpose() * W * r;
        }
        A.diagonal().array() += 1e-9;
        const Eigen::Vector4d dx = A.ldlt().solve(b);
        if (!dx.allFinite()) {
            return;
        }
        h.yaw = std::atan2(std::sin(h.yaw + dx(0)), std::cos(h.yaw + dx(0)));
        h.t += dx.tail<3>();
        if (dx.norm() < 1e-6) {
            break;
        }
    }
}

/// Members dropped (worst first) until the whole placement passes.
void enforce_joint_fit(const StructureTemplate& tmpl,
                       const StructureVariant& v,
                       const std::vector<FitLandmark>& lms,
                       Hypothesis& h) {
    while (static_cast<int>(h.members.size()) >= tmpl.min_members) {
        const int dof = std::max(1, 3 * static_cast<int>(h.members.size()) - 4);
        const Eigen::Isometry3d T = make_pose(h.yaw, h.t);
        double cost = 0.0;
        double worst = -1.0;
        std::size_t worst_i = 0;
        for (std::size_t k = 0; k < h.members.size(); ++k) {
            const double c = member_chi2(v.members[h.members[k].first], T,
                                         lms[h.members[k].second]);
            cost += c;
            if (c > worst) {
                worst = c;
                worst_i = k;
            }
        }
        h.cost = cost;
        if (cost <= chi2_quantile(tmpl.fit_probability, dof)) {
            return;
        }
        h.members.erase(h.members.begin() + static_cast<std::ptrdiff_t>(worst_i));
        refine(v, lms, h);
    }
}

bool better(const Hypothesis& a, const Hypothesis& b) {
    if (a.members.size() != b.members.size()) {
        return a.members.size() > b.members.size();
    }
    return a.cost < b.cost;
}

std::optional<Hypothesis> best_for_variant(const StructureTemplate& tmpl,
                                           const StructureVariant& v,
                                           const std::vector<FitLandmark>& lms,
                                           const std::optional<FitPrior>& prior,
                                           int min_members) {
    std::optional<Hypothesis> best;
    for (std::size_t s1 = 0; s1 < v.members.size(); ++s1) {
        for (std::size_t s2 = 0; s2 < v.members.size(); ++s2) {
            const Eigen::Vector3d off = v.members[s2].offset - v.members[s1].offset;
            if (s1 == s2 || off.head<2>().norm() < kMinBaseline) {
                continue;
            }
            for (std::size_t a = 0; a < lms.size(); ++a) {
                if (!v.members[s1].accepts(lms[a].key)) {
                    continue;
                }
                for (std::size_t b = 0; b < lms.size(); ++b) {
                    if (a == b || !v.members[s2].accepts(lms[b].key)) {
                        continue;
                    }
                    const Eigen::Vector3d d = lms[b].position - lms[a].position;
                    const double tol =
                        3.0 * (v.members[s1].sigma.head<2>().norm() +
                               v.members[s2].sigma.head<2>().norm()) +
                        3.0 * std::sqrt(lms[a].covariance.trace() +
                                        lms[b].covariance.trace());
                    if (std::abs(d.head<2>().norm() - off.head<2>().norm()) >
                        tol) {
                        continue;
                    }
                    Hypothesis h;
                    h.yaw = std::atan2(d.y(), d.x()) - std::atan2(off.y(), off.x());
                    const Eigen::Matrix3d R = yaw_rotation(h.yaw);
                    h.t = 0.5 * (lms[a].position - R * v.members[s1].offset +
                                 lms[b].position - R * v.members[s2].offset);
                    assign(tmpl, v, lms, h);
                    refine(v, lms, h);
                    assign(tmpl, v, lms, h);
                    refine(v, lms, h);
                    enforce_joint_fit(tmpl, v, lms, h);
                    if (static_cast<int>(h.members.size()) < min_members) {
                        continue;
                    }
                    if (prior && !prior->allows(make_pose(h.yaw, h.t))) {
                        continue;
                    }
                    if (!best || better(h, *best)) {
                        best = std::move(h);
                    }
                }
            }
        }
    }
    return best;
}

}  // namespace

double chi2_quantile(double p, int dof) {
    const boost::math::chi_squared dist(static_cast<double>(std::max(1, dof)));
    return boost::math::quantile(dist, std::clamp(p, 1e-9, 1.0 - 1e-12));
}

bool StructureMember::accepts(const LandmarkClassKey& key) const {
    return std::any_of(classes.begin(), classes.end(), [&](const auto& c) {
        return c.type == key.type && (c.subtype == 0 || c.subtype == key.subtype);
    });
}

bool StructureTemplate::has_member_class(const LandmarkClassKey& key) const {
    for (const auto& v : variants) {
        for (const auto& m : v.members) {
            if (m.accepts(key)) {
                return true;
            }
        }
    }
    return false;
}

std::vector<StructureTemplate> parse_structures(const YAML::Node& node) {
    std::vector<StructureTemplate> out;
    if (!node) {
        return out;
    }
    for (const auto& kv : node) {
        StructureTemplate t;
        t.name = kv.first.as<std::string>();
        const std::string where = "course.templates." + t.name;
        const YAML::Node& n = kv.second;
        if (n["min_members"]) {
            t.min_members = n["min_members"].as<int>();
        }
        if (n["member_gate_chi2"]) {
            t.member_gate_chi2 = n["member_gate_chi2"].as<double>();
        }
        if (n["fit_probability"]) {
            t.fit_probability = n["fit_probability"].as<double>();
        }
        if (n["variant_margin_chi2"]) {
            t.variant_margin_chi2 = n["variant_margin_chi2"].as<double>();
        }
        if (n["balanced_classes"]) {
            t.balanced_classes = n["balanced_classes"].as<bool>();
        }
        const Eigen::Vector3d sigma = n["sigma"]
                                          ? parse_vec3(n["sigma"], where + ".sigma")
                                          : Eigen::Vector3d(0.2, 0.2, 0.3);
        if (n["variants"]) {
            for (const auto& v : n["variants"]) {
                StructureVariant var;
                var.name = v.first.as<std::string>();
                check_keys(v.second, {"members", "points"},
                           where + ".variants." + var.name);
                var.members = parse_members(v.second["members"], sigma,
                                            where + ".variants." + var.name);
                var.points = parse_points(v.second["points"], var.members,
                                          where + ".variants." + var.name + ".points");
                // Points of the whole template go with every variant.
                auto common = parse_points(n["points"], var.members, where + ".points");
                var.points.insert(var.points.end(), common.begin(), common.end());
                t.variants.push_back(std::move(var));
            }
        } else {
            StructureVariant var;
            var.name = "default";
            var.members = parse_members(n["members"], sigma, where + ".members");
            var.points = parse_points(n["points"], var.members, where + ".points");
            t.variants.push_back(std::move(var));
        }
        for (const auto& v : t.variants) {
            if (t.min_members < 2 ||
                t.min_members > static_cast<int>(v.members.size())) {
                throw std::runtime_error(where + ".min_members must be 2.." +
                                         std::to_string(v.members.size()));
            }
        }
        out.push_back(std::move(t));
    }
    return out;
}

double member_chi2(const StructureMember& m,
                   const Eigen::Isometry3d& pose,
                   const FitLandmark& lm) {
    const Eigen::Vector3d r = lm.position - pose * m.offset;
    return r.dot(member_cov(m, pose.linear(), lm).ldlt().solve(r));
}

bool FitPrior::allows(const Eigen::Isometry3d& pose) const {
    if ((pose.translation().head<2>() - center).norm() > radius) {
        return false;
    }
    const double yaw_dev = std::abs(std::remainder(yaw_of(pose) - yaw, 2.0 * M_PI));
    if (yaw_dev <= yaw_window) {
        return true;
    }
    return symmetric && std::abs(M_PI - yaw_dev) <= yaw_window;
}

std::optional<StructureFit> fit_structure(const StructureTemplate& tmpl,
                                          const std::vector<FitLandmark>& free,
                                          const std::optional<FitPrior>& prior,
                                          int min_members) {
    const int need = min_members > 0 ? min_members : tmpl.min_members;
    std::vector<std::pair<std::size_t, Hypothesis>> bests;
    for (std::size_t vi = 0; vi < tmpl.variants.size(); ++vi) {
        if (auto h = best_for_variant(tmpl, tmpl.variants[vi], free, prior,
                                      need)) {
            bests.emplace_back(vi, std::move(*h));
        }
    }
    if (bests.empty()) {
        return std::nullopt;
    }
    std::sort(bests.begin(), bests.end(), [](const auto& a, const auto& b) {
        return better(a.second, b.second);
    });
    if (bests.size() > 1 &&
        bests[0].second.members.size() == bests[1].second.members.size() &&
        bests[1].second.cost - bests[0].second.cost < tmpl.variant_margin_chi2) {
        return std::nullopt;  // the variants cannot be told apart yet
    }
    const auto& [vi, h] = bests.front();
    StructureFit fit;
    fit.variant = vi;
    fit.pose = make_pose(h.yaw, h.t);
    fit.cost = h.cost;
    for (const auto& [mi, li] : h.members) {
        fit.members.emplace_back(mi, free[li].id);
    }
    return fit;
}

Eigen::Isometry3d refit_pose(
    const StructureVariant& variant,
    const Eigen::Isometry3d& pose,
    const std::vector<std::pair<std::size_t, FitLandmark>>& members) {
    if (members.size() < 2) {
        return pose;
    }
    std::vector<FitLandmark> lms;
    Hypothesis h;
    h.yaw = yaw_of(pose);
    h.t = pose.translation();
    for (const auto& [mi, lm] : members) {
        h.members.emplace_back(mi, lms.size());
        lms.push_back(lm);
    }
    refine(variant, lms, h);
    return make_pose(h.yaw, h.t);
}

}  // namespace vortex::mission
