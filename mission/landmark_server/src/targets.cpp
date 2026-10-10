#include "landmark_server/targets.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

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

std::vector<NamedPose> gate_frames(const std::vector<LandmarkState>& landmarks,
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
    const gtsam::Point3 along = b->pose.translation() - a->pose.translation();
    const double separation = std::hypot(along.x(), along.y());
    if (separation < gate.min_separation_m ||
        separation > gate.max_separation_m) {
        return {};
    }
    const gtsam::Point3 middle =
        0.5 * (a->pose.translation() + b->pose.translation());
    double yaw = std::atan2(along.x(), -along.y());
    const gtsam::Point3 to_middle = middle - start;
    if (std::cos(yaw) * to_middle.x() + std::sin(yaw) * to_middle.y() < 0.0) {
        yaw += M_PI;
    }
    const gtsam::Rot3 R = gtsam::Rot3::Yaw(yaw);
    const gtsam::Point3 through(std::cos(yaw), std::sin(yaw), 0.0);
    const gtsam::Point3 down(0.0, 0.0, gate.depth_below_panel_m);
    std::vector<NamedPose> out{{"gate_middle", gtsam::Pose3(R, middle)}};
    for (const LandmarkState* panel : {a, b}) {
        const gtsam::Point3 p = panel->pose.translation() + down;
        out.push_back({panel->cls.name + "_entrance",
                       gtsam::Pose3(R, p - gate.approach_m * through)});
        out.push_back({panel->cls.name + "_exit",
                       gtsam::Pose3(R, p + gate.approach_m * through)});
    }
    return out;
}

std::vector<NamedPose> slalom_frames(
    const std::vector<LandmarkState>& landmarks,
    const SlalomParams& slalom,
    const gtsam::Point3& start) {
    if (slalom.red_class.empty()) {
        return {};
    }
    using Vec2 = Eigen::Vector2d;
    const auto xy = [](const LandmarkState& l) {
        return Vec2(l.pose.x(), l.pose.y());
    };
    // Each red pipe is a row, nearest the start first.
    std::vector<const LandmarkState*> reds;
    std::vector<const LandmarkState*> whites;
    for (const LandmarkState& l : landmarks) {
        if (l.cls.name == slalom.red_class) {
            reds.push_back(&l);
        } else if (l.cls.name == slalom.white_class) {
            whites.push_back(&l);
        }
    }
    const Vec2 origin(start.x(), start.y());
    std::sort(reds.begin(), reds.end(),
              [&](const LandmarkState* a, const LandmarkState* b) {
                  return (xy(*a) - origin).norm() < (xy(*b) - origin).norm();
              });

    std::vector<NamedPose> out;
    Vec2 through(1.0, 0.0);  // the start heading until a row gives its own
    int last_row = -1;
    for (std::size_t k = 0; k < reds.size(); ++k) {
        const Vec2 red = xy(*reds[k]);
        // The row's number: how many row spacings behind the first row, so a
        // row that is not mapped yet leaves its number free instead of
        // shifting the rows behind it.
        const int n = static_cast<int>(std::lround(
            (red - xy(*reds[0])).dot(through) / slalom.row_spacing_m));
        if (n <= last_row) {
            continue;  // a second red pipe in a row: a false one
        }
        last_row = n;
        // y is to the right of x (z down): right of the way through.
        const auto right_of = [](const Vec2& t) { return Vec2(-t.y(), t.x()); };
        // The row's white pipes: the nearest one on each side within the
        // spacing limits, not ahead of or behind the red pipe.
        std::optional<Vec2> white[2];  // 0 = left, 1 = right
        for (const LandmarkState* w : whites) {
            const Vec2 d = xy(*w) - red;
            const double across = d.dot(right_of(through));
            const double along = d.dot(through);
            if (d.norm() < slalom.min_spacing_m ||
                d.norm() > slalom.max_spacing_m ||
                std::abs(along) > std::abs(across)) {
                continue;
            }
            auto& slot = white[across > 0.0 ? 1 : 0];
            if (!slot || d.norm() < (*slot - red).norm()) {
                slot = xy(*w);
            }
        }
        // Straight through the row: across the line of its pipes.
        if (white[0] || white[1]) {
            const Vec2 line =
                (white[1] ? *white[1] : red) - (white[0] ? *white[0] : red);
            Vec2 t(line.y(), -line.x());
            t.normalize();
            through = t.dot(red - origin) < 0.0 ? Vec2(-t) : t;
        }
        const gtsam::Rot3 R =
            gtsam::Rot3::Yaw(std::atan2(through.y(), through.x()));
        const Vec2 right = right_of(through);
        for (const int side : {0, 1}) {
            const Vec2 p =
                white[side] ? Vec2(0.5 * (red + *white[side]))
                            : Vec2(red + (side == 1 ? 0.5 : -0.5) *
                                             slalom.nominal_spacing_m * right);
            out.push_back(
                {std::string(side == 1 ? "slalom_right_" : "slalom_left_") +
                     std::to_string(n),
                 gtsam::Pose3(R,
                              gtsam::Point3(p.x(), p.y(), reds[k]->pose.z()))});
        }
    }
    return out;
}

}  // namespace vortex::landmark_server
