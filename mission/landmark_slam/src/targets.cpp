#include "landmark_slam/targets.hpp"

#include <cmath>
#include <optional>

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

}  // namespace vortex::landmark_slam
