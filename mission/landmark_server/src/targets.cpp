#include "landmark_server/targets.hpp"

#include <algorithm>
#include <cmath>

namespace vortex::landmark_server {

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

namespace {

void add_passage(std::vector<NamedPose>& out,
                 const Landmark& panel,
                 double yaw,
                 const GateParams& gate) {
    const Eigen::Vector3d through(std::cos(yaw), std::sin(yaw), 0.0);
    const Eigen::Vector3d p =
        panel.position + Eigen::Vector3d(0.0, 0.0, gate.depth_below_panel_m);
    out.push_back({panel.cls.name + "_entrance",
                   yaw_pose(yaw, p - gate.approach_m * through)});
    out.push_back({panel.cls.name + "_exit",
                   yaw_pose(yaw, p + gate.approach_m * through)});
}

}  // namespace

std::vector<NamedPose> gate_frames(const std::vector<Landmark>& landmarks,
                                   const GateParams& gate,
                                   const Eigen::Vector3d& start) {
    if (gate.panel_classes.size() != 2) {
        return {};
    }
    const Landmark* a = best_of(landmarks, gate.panel_classes[0]);
    const Landmark* b = best_of(landmarks, gate.panel_classes[1]);
    std::vector<NamedPose> out;
    if (a && b) {
        const Eigen::Vector3d along = b->position - a->position;
        const double separation = std::hypot(along.x(), along.y());
        if (separation >= gate.min_separation_m &&
            separation <= gate.max_separation_m) {
            const Eigen::Vector3d middle = 0.5 * (a->position + b->position);
            double yaw = std::atan2(along.x(), -along.y());
            const Eigen::Vector3d to_middle = middle - start;
            if (std::cos(yaw) * to_middle.x() + std::sin(yaw) * to_middle.y() <
                0.0) {
                yaw += M_PI;
            }
            out.push_back({"gate_middle", yaw_pose(yaw, middle)});
            add_passage(out, *a, yaw, gate);
            add_passage(out, *b, yaw, gate);
            return out;
        }
    }
    // One panel, or two that cannot be one gate: drive the line from the
    // start through each. Right only if the start is in front of the gate.
    for (const Landmark* panel : {a, b}) {
        if (panel) {
            const Eigen::Vector3d d = panel->position - start;
            add_passage(out, *panel, std::atan2(d.y(), d.x()), gate);
        }
    }
    return out;
}

}  // namespace vortex::landmark_server
