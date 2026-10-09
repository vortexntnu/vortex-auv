#include "landmark_slam/targets.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace vortex::landmark_slam {

namespace {

/// The most observed landmark of the class, or nullptr.
const LandmarkState* best_of(const std::vector<LandmarkState>& landmarks,
                             const std::string& cls) {
    const LandmarkState* best = nullptr;
    for (const auto& l : landmarks) {
        if (l.cls.name == cls && (!best || l.n_obs > best->n_obs)) {
            best = &l;
        }
    }
    return best;
}

constexpr double kMinIconSpreadM = 0.2;

using IconOpenings =
    std::vector<std::pair<const LandmarkState*, const std::vector<double>*>>;

/// The board's right axis from the icons when the detector gives no
/// normal: their horizontal principal direction (they spread along it).
std::optional<gtsam::Point3> right_from_icons(const IconOpenings& icons) {
    if (icons.size() < 2) {
        return std::nullopt;
    }
    gtsam::Vector2 mean = gtsam::Vector2::Zero();
    for (const auto& [l, _] : icons) {
        mean += l->pose.translation().head<2>();
    }
    mean /= static_cast<double>(icons.size());
    gtsam::Matrix2 C = gtsam::Matrix2::Zero();
    for (const auto& [l, _] : icons) {
        const gtsam::Vector2 d = l->pose.translation().head<2>() - mean;
        C += d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<gtsam::Matrix2> eig(C);
    const gtsam::Point3 right(eig.eigenvectors()(0, 1),
                              eig.eigenvectors()(1, 1), 0.0);
    double lo = 0.0;
    double hi = 0.0;
    for (const auto& [l, _] : icons) {
        const double s =
            right.head<2>().dot(l->pose.translation().head<2>() - mean);
        lo = std::min(lo, s);
        hi = std::max(hi, s);
    }
    if (hi - lo < kMinIconSpreadM) {
        return std::nullopt;
    }
    return right;
}

}  // namespace

std::vector<TargetFrame> gate_frames(
    const std::vector<LandmarkState>& landmarks,
    const GateParams& gate,
    const gtsam::Point3& start) {
    if (gate.panel_classes.size() != 2) {
        return {};
    }
    const LandmarkState* a = best_of(landmarks, gate.panel_classes[0]);
    const LandmarkState* b = best_of(landmarks, gate.panel_classes[1]);
    if (!a || !b) {
        return {};
    }
    const gtsam::Point3 pa = a->pose.translation();
    const gtsam::Point3 pb = b->pose.translation();
    const gtsam::Point3 along = pb - pa;
    const double separation = std::hypot(along.x(), along.y());
    if (separation < gate.min_separation_m ||
        separation > gate.max_separation_m) {
        return {};
    }
    // Normal to the panel line, pointing away from the start side.
    const gtsam::Point3 middle = 0.5 * (pa + pb);
    double yaw = std::atan2(along.x(), -along.y());
    const gtsam::Point3 to_middle = middle - start;
    if (std::cos(yaw) * to_middle.x() + std::sin(yaw) * to_middle.y() < 0.0) {
        yaw += M_PI;
    }
    const gtsam::Rot3 R = gtsam::Rot3::Yaw(yaw);
    const gtsam::Point3 through(std::cos(yaw), std::sin(yaw), 0.0);
    const gtsam::Point3 down(0.0, 0.0, gate.depth_below_panel_m);

    std::vector<TargetFrame> out{{"gate_middle", gtsam::Pose3(R, middle)}};
    for (const LandmarkState* panel : {a, b}) {
        const gtsam::Point3 p = panel->pose.translation() + down;
        out.push_back({panel->cls.name + "_entrance",
                       gtsam::Pose3(R, p - gate.approach_m * through)});
        out.push_back({panel->cls.name + "_exit",
                       gtsam::Pose3(R, p + gate.approach_m * through)});
    }
    return out;
}

std::vector<TargetFrame> torpedo_frames(
    const std::vector<LandmarkState>& landmarks,
    const TorpedoParams& torpedo,
    const gtsam::Point3& vehicle) {
    if (torpedo.board_class.empty()) {
        return {};
    }
    const LandmarkState* board = best_of(landmarks, torpedo.board_class);
    if (!board) {
        return {};
    }
    // Icon -> the openings of its size.
    IconOpenings icons;
    for (const auto& [names, openings] :
         {std::pair{&torpedo.large_icons, &torpedo.large_openings},
          std::pair{&torpedo.small_icons, &torpedo.small_openings}}) {
        for (const auto& name : *names) {
            if (const LandmarkState* l = best_of(landmarks, name)) {
                icons.emplace_back(l, openings);
            }
        }
    }
    if (icons.empty()) {
        return {};
    }
    const gtsam::Point3 down(0.0, 0.0, 1.0);
    gtsam::Point3 through;
    if (board->yaw_known) {
        // The detector's normal: the board frame's x axis, levelled.
        const gtsam::Point3 x = board->pose.rotation().r1();
        through = gtsam::Point3(x.x(), x.y(), 0.0).normalized();
    } else {
        const auto axis = right_from_icons(icons);
        if (!axis) {
            return {};
        }
        through = axis->cross(down);
    }
    if (through.dot(board->pose.translation() - vehicle) < 0.0) {
        through = -through;
    }
    const gtsam::Point3 right = down.cross(through);
    const gtsam::Rot3 R(through, right, down);
    const gtsam::Point3 centre = board->pose.translation();

    std::vector<TargetFrame> out;
    for (const auto& [icon, openings] : icons) {
        const gtsam::Point3 rel = R.unrotate(icon->pose.translation() - centre);
        std::optional<gtsam::Point3> best;
        for (std::size_t k = 0; k + 1 < openings->size(); k += 2) {
            const gtsam::Point3 o(0.0, (*openings)[k], (*openings)[k + 1]);
            if (!best || (o - rel).norm() < (*best - rel).norm()) {
                best = o;
            }
        }
        if (best) {
            out.push_back({icon->cls.name + "_opening",
                           gtsam::Pose3(R, centre + R.rotate(*best))});
        }
    }
    return out;
}

}  // namespace vortex::landmark_slam
