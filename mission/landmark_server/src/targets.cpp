#include "landmark_server/targets.hpp"

#include <algorithm>
#include <cmath>

namespace vortex::landmark_server {

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

namespace {

void add_passage(std::vector<NamedPose>& out,
                 const LandmarkState& panel,
                 double yaw,
                 const GateParams& gate) {
    const gtsam::Rot3 R = gtsam::Rot3::Yaw(yaw);
    const gtsam::Point3 through(std::cos(yaw), std::sin(yaw), 0.0);
    const gtsam::Point3 p = panel.pose.translation() +
                            gtsam::Point3(0.0, 0.0, gate.depth_below_panel_m);
    out.push_back({panel.cls.name + "_entrance",
                   gtsam::Pose3(R, p - gate.approach_m * through)});
    out.push_back({panel.cls.name + "_exit",
                   gtsam::Pose3(R, p + gate.approach_m * through)});
}

}  // namespace

std::vector<NamedPose> gate_frames(const std::vector<LandmarkState>& landmarks,
                                   const GateParams& gate,
                                   const gtsam::Point3& start) {
    if (gate.panel_classes.size() != 2) {
        return {};
    }
    const LandmarkState* a = best_of(landmarks, gate.panel_classes[0]);
    const LandmarkState* b = best_of(landmarks, gate.panel_classes[1]);
    std::vector<NamedPose> out;
    if (a && b) {
        const gtsam::Point3 along =
            b->pose.translation() - a->pose.translation();
        const double separation = std::hypot(along.x(), along.y());
        if (separation >= gate.min_separation_m &&
            separation <= gate.max_separation_m) {
            const gtsam::Point3 middle =
                0.5 * (a->pose.translation() + b->pose.translation());
            double yaw = std::atan2(along.x(), -along.y());
            const gtsam::Point3 to_middle = middle - start;
            if (std::cos(yaw) * to_middle.x() + std::sin(yaw) * to_middle.y() <
                0.0) {
                yaw += M_PI;
            }
            out.push_back(
                {"gate_middle", gtsam::Pose3(gtsam::Rot3::Yaw(yaw), middle)});
            add_passage(out, *a, yaw, gate);
            add_passage(out, *b, yaw, gate);
            return out;
        }
    }
    // One panel, or two that cannot be one gate: drive the line from the
    // start through each. Right only if the start is in front of the gate.
    for (const LandmarkState* panel : {a, b}) {
        if (panel) {
            const gtsam::Point3 d = panel->pose.translation() - start;
            add_passage(out, *panel, std::atan2(d.y(), d.x()), gate);
        }
    }
    return out;
}

}  // namespace vortex::landmark_server
