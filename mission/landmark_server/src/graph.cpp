#include "landmark_server/graph.hpp"

#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>

namespace vortex::landmark_server {

namespace {

using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using gtsam::noiseModel::Robust;
using gtsam::noiseModel::mEstimator::DCS;

constexpr double kAnchorSigma = 1e-3;
constexpr double kMinRangeM = 0.05;

gtsam::Key X(int kf) {
    return gtsam::Symbol('x', static_cast<std::uint64_t>(kf));
}

gtsam::Key L(int id) {
    return gtsam::Symbol('l', static_cast<std::uint64_t>(id));
}

gtsam::Pose3 to_gtsam(const Pose& T) {
    return gtsam::Pose3(gtsam::Rot3(Eigen::Matrix3d(T.rotation())),
                        gtsam::Point3(T.translation()));
}

Pose to_pose(const gtsam::Pose3& T) {
    Pose out = Pose::Identity();
    out.linear() = T.rotation().matrix();
    out.translation() = T.translation();
    return out;
}

/// Depth from the pressure sensor: constrains only z of the pose.
class DepthFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
   public:
    DepthFactor(gtsam::Key key, double z, const gtsam::SharedNoiseModel& model)
        : NoiseModelFactorN(model, key), z_(z) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        boost::optional<gtsam::Matrix&> H = boost::none) const override {
        gtsam::Matrix36 H_t;
        const gtsam::Point3 t = pose.translation(H ? &H_t : nullptr);
        if (H) {
            *H = H_t.row(2);
        }
        return gtsam::Vector1(t.z() - z_);
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::make_shared<DepthFactor>(*this);
    }

   private:
    double z_;
};

}  // namespace

struct DriftGraph::Impl {
    struct Pending {
        Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
        int n{0};
    };

    GraphParams params;
    std::unique_ptr<gtsam::ISAM2> isam;
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    gtsam::Values estimate;
    std::optional<gtsam::Marginals> marginals;  // of this estimate, when asked
    gtsam::Pose3 last_odom;
    Eigen::Vector3d last_position{Eigen::Vector3d::Zero()};
    double last_t{0.0};
    int last_kf{0};
    std::map<int, Pending> pending;

    void add_absolute_factors(int kf, const gtsam::Pose3& T_odom_base) {
        factors.emplace_shared<DepthFactor>(
            X(kf), T_odom_base.z(), Isotropic::Sigma(1, params.depth_sigma));
        // Gravity seen in the base frame.
        const gtsam::Unit3 b_gravity(
            T_odom_base.rotation().unrotate(gtsam::Point3(0.0, 0.0, 1.0)));
        factors.emplace_shared<gtsam::Pose3AttitudeFactor>(
            X(kf), gtsam::Unit3(0.0, 0.0, 1.0),
            Isotropic::Sigma(2, params.attitude_sigma), b_gravity);
    }

    void solve() {
        isam->update(factors, values);
        factors.resize(0);
        values.clear();
        estimate = isam->calculateEstimate();
        marginals.reset();
    }

    /// From the Bayes tree, which does not grow with the graph like Marginals.
    /// It can not answer for every pair of cliques, then Marginals does.
    std::optional<Eigen::Matrix<double, 9, 9>> pose_landmark_cov(gtsam::Key x,
                                                                 gtsam::Key l) {
        try {
            const auto pair = isam->joint(x, l);
            return Eigen::Matrix<double, 9, 9>(
                pair->hessian(gtsam::Ordering{x, l}).first.inverse());
        } catch (const std::exception&) {
            const auto P = joint({x, l});
            if (!P) {
                return std::nullopt;
            }
            Eigen::Matrix<double, 9, 9> out;
            out << (*P)(x, x), (*P)(x, l), (*P)(l, x), (*P)(l, l);
            return out;
        }
    }

    /// nullopt when the graph can not be factored, then nothing is matched.
    std::optional<gtsam::JointMarginal> joint(const gtsam::KeyVector& keys) {
        try {
            if (!marginals) {
                marginals.emplace(isam->getFactorsUnsafe(), estimate);
            }
            return marginals->jointMarginalCovariance(keys);
        } catch (const gtsam::IndeterminantLinearSystemException&) {
            return std::nullopt;
        }
    }

    void add_keyframe(const Pose& T_odom_base) {
        const GraphParams& p = params;
        const gtsam::Pose3 odom = to_gtsam(T_odom_base);
        const gtsam::Pose3 delta = last_odom.between(odom);
        const double dist = delta.translation().norm();
        const double trans =
            std::max(p.min_sigma_trans, p.sigma_trans_per_m * dist);
        const double yaw = std::max(p.min_sigma_yaw, p.sigma_yaw_per_m * dist);
        const int kf = last_kf + 1;

        // Pose3 tangent order: roll, pitch, yaw, x, y, z.
        factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(last_kf), X(kf), delta,
            Diagonal::Sigmas((gtsam::Vector6() << p.sigma_roll_pitch,
                              p.sigma_roll_pitch, yaw, trans, trans, p.sigma_z)
                                 .finished()));
        const gtsam::Pose3 guess =
            estimate.at<gtsam::Pose3>(X(last_kf)) * delta;
        values.insert(X(kf), guess);
        add_absolute_factors(kf, odom);

        // Each detection is in odom, through the odom pose of its image. Here
        // they go to this keyframe's base frame with its odom pose.
        const Pose T_base_odom = T_odom_base.inverse();
        for (const auto& [id, q] : pending) {
            const Eigen::Vector3d z =
                T_base_odom * Eigen::Vector3d(q.sum / q.n);
            const double range = z.norm();
            if (range < kMinRangeM) {
                continue;
            }
            if (!estimate.exists(L(id))) {
                values.insert(L(id), gtsam::Point3(guess.transformFrom(z)));
            }
            factors.emplace_shared<
                gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(
                X(kf), L(id), gtsam::Unit3(gtsam::Point3(z)), range,
                Robust::Create(
                    DCS::Create(p.dcs_phi),
                    Diagonal::Sigmas(gtsam::Vector3(
                        p.bearing_sigma, p.bearing_sigma,
                        p.range_sigma_a + p.range_sigma_b * range))));
        }
        pending.clear();
        last_kf = kf;
        last_odom = odom;
        solve();
    }
};

DriftGraph::DriftGraph() : impl_(std::make_unique<Impl>()) {}

DriftGraph::~DriftGraph() = default;

void DriftGraph::reset(const GraphParams& params,
                       const Pose& T_map_start,
                       const Pose& T_odom_base,
                       double t) {
    Impl& g = *impl_;
    g.params = params;
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
    g.isam = std::make_unique<gtsam::ISAM2>(isam_params);
    g.factors.resize(0);
    g.values.clear();
    g.pending.clear();
    g.last_kf = 0;
    g.last_t = t;
    g.last_odom = to_gtsam(T_odom_base);
    g.last_position = T_odom_base.translation();

    const gtsam::Pose3 X0 = to_gtsam(T_map_start);
    g.factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(0), X0, Isotropic::Sigma(6, kAnchorSigma));
    g.values.insert(X(0), X0);
    g.add_absolute_factors(0, g.last_odom);
    g.solve();
}

void DriftGraph::update(const Pose& T_odom_base,
                        double t,
                        const std::vector<Observation>& observations) {
    Impl& g = *impl_;
    for (const Observation& o : observations) {
        auto& q = g.pending[o.id];
        q.sum += o.position;
        q.n++;
    }
    const bool moved = (T_odom_base.translation() - g.last_position).norm() >=
                       g.params.keyframe_dist_m;
    const bool seen =
        !g.pending.empty() && t - g.last_t >= g.params.keyframe_time_s;
    if (moved || seen) {
        g.add_keyframe(T_odom_base);
        g.last_position = T_odom_base.translation();
        g.last_t = t;
    }
}

Pose DriftGraph::T_map_odom() const {
    const gtsam::Pose3 X_k =
        impl_->estimate.at<gtsam::Pose3>(X(impl_->last_kf));
    return to_pose(X_k * impl_->last_odom.inverse());
}

std::optional<Eigen::Vector3d> DriftGraph::landmark(int id) const {
    if (!impl_->estimate.exists(L(id))) {
        return std::nullopt;
    }
    return impl_->estimate.at<gtsam::Point3>(L(id));
}

Eigen::MatrixXd DriftGraph::squared_distances(
    const std::vector<Eigen::Vector3d>& detections,
    const std::vector<int>& ids) const {
    Impl& g = *impl_;
    Eigen::MatrixXd out =
        Eigen::MatrixXd::Constant(static_cast<Eigen::Index>(detections.size()),
                                  static_cast<Eigen::Index>(ids.size()),
                                  std::numeric_limits<double>::infinity());
    const gtsam::Key x = X(g.last_kf);
    if (detections.empty()) {
        return out;
    }
    std::vector<std::optional<Eigen::Matrix<double, 9, 9>>> cov;
    for (const int id : ids) {
        cov.push_back(g.estimate.exists(L(id)) ? g.pose_landmark_cov(x, L(id))
                                               : std::nullopt);
    }
    const gtsam::Pose3 X_k = g.estimate.at<gtsam::Pose3>(x);
    const Pose T_base_odom = to_pose(g.last_odom).inverse();
    const auto unit = gtsam::noiseModel::Unit::Create(3);
    for (Eigen::Index r = 0; r < out.rows(); ++r) {
        const Eigen::Vector3d z = T_base_odom * detections[r];
        const double range = z.norm();
        if (range < kMinRangeM) {
            continue;
        }
        const double s =
            g.params.range_sigma_a + g.params.range_sigma_b * range;
        const Eigen::Matrix3d R =
            gtsam::Vector3(g.params.bearing_sigma, g.params.bearing_sigma, s)
                .cwiseAbs2()
                .asDiagonal();
        for (Eigen::Index c = 0; c < out.cols(); ++c) {
            if (!cov[c]) {
                continue;
            }
            const gtsam::Key l = L(ids[c]);
            const gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3> factor(
                x, l, gtsam::Unit3(gtsam::Point3(z)), range, unit);
            gtsam::Matrix H_x;
            gtsam::Matrix H_l;
            const gtsam::Vector nu = factor.evaluateError(
                X_k, g.estimate.at<gtsam::Point3>(l), H_x, H_l);
            Eigen::Matrix<double, 3, 9> H;
            H << H_x, H_l;
            const Eigen::Matrix3d S = H * (*cov[c]) * H.transpose() + R;
            out(r, c) = nu.dot(S.ldlt().solve(nu));
        }
    }
    return out;
}

double DriftGraph::squared_distance_between(int a, int b) const {
    Impl& g = *impl_;
    const gtsam::Key la = L(a);
    const gtsam::Key lb = L(b);
    const auto P = g.joint({la, lb});
    if (!P) {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::Vector3d d =
        g.estimate.at<gtsam::Point3>(la) - g.estimate.at<gtsam::Point3>(lb);
    const Eigen::Matrix3d S =
        (*P)(la, la) + (*P)(lb, lb) - (*P)(la, lb) - (*P)(lb, la);
    return d.dot(S.ldlt().solve(d));
}

}  // namespace vortex::landmark_server
