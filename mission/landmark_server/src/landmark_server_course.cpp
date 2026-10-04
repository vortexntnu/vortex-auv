#include <spdlog/spdlog.h>
#include <algorithm>
#include "landmark_server/landmark_server_ros.hpp"

namespace vortex::mission {

CourseGeometry LandmarkServerNode::course_geometry() const {
    CourseGeometry g;
    g.set = course_->status() != CourseFrameStatus::UNSET;
    g.at_gate = course_->status() == CourseFrameStatus::GATE_LOCKED;
    g.through_yaw = course_->through_yaw();
    const CourseFrameTracker* c = course_.get();
    g.to_odom = [c](const Eigen::Vector2d& v) { return c->from_course(v); };
    g.to_course = [c](const Eigen::Vector2d& v) { return c->to_course(v); };
    return g;
}

void LandmarkServerNode::gate_measurements(std::vector<Landmark>& measurements) {
    const CourseModel& course = map_->course();
    const CourseGeometry geo = course_geometry();
    const bool lane = course_->status() != CourseFrameStatus::UNSET;
    std::optional<Eigen::Vector3d> vehicle;
    {
        std::lock_guard<std::mutex> lock(odom_mtx_);
        if (last_odom_position_) {
            vehicle = Eigen::Vector3d(last_odom_position_->x, last_odom_position_->y,
                                      last_odom_position_->z);
        }
    }
    std::vector<Landmark> kept;
    kept.reserve(measurements.size());
    for (auto& m : measurements) {
        m.reported_class = m.class_key;
        const Eigen::Vector3d p = m.pose.pos_vector();
        // The lane: the area the course layout covers, else the boxes of
        // course_frame.lane.
        const bool in_lane = course.enabled() ? course.lane_allows(p, geo)
                                              : course_->position_allowed(p);
        if (lane && !in_lane) {
            ++drop_counts_["outside_lane"];
            continue;
        }
        if (course.enabled()) {
            m.class_key = course.config().kind_of(m.class_key);
            if (const auto reason = course.intake_reject(m.class_key, p, geo, vehicle)) {
                ++drop_counts_[*reason];
                continue;
            }
        }
        kept.push_back(std::move(m));
    }
    measurements = std::move(kept);

    // Without a course frame no task can be mapped: say so now and then.
    if (course.enabled() && !geo.set && ++course_warn_ticks_ % 50 == 1) {
        spdlog::warn(
            "LandmarkServer: the course frame is not set: no task is mapped "
            "(call landmark_server/set_course_frame)");
    }
}

void LandmarkServerNode::apply_track_limits() {
    auto& per_class = track_manager_config_.per_class_configs;
    const auto entry_for =
        [&](const LandmarkClassKey& key) -> vortex::filtering::LandmarkClassConfig& {
        for (auto& [k, cfg] : per_class) {
            if (k == key) {
                return cfg;
            }
        }
        per_class.emplace_back(key, track_manager_config_.default_class_config);
        return per_class.back().second;
    };
    const CourseConfig& course = map_config_.course;
    const auto templated = course.templated_kinds();
    for (const auto& [kind, n] : map_->course().track_limits()) {
        entry_for({kind.first, kind.second}).max_tracks = n;
    }
    for (const auto& [key, rule] : map_config_.class_rules) {
        if (key.second == 0) {
            continue;
        }
        const LandmarkClassKey k{key.first, key.second};
        const auto kind = course.enable ? course.kind_of(k) : k;
        if (templated.contains({kind.type, kind.subtype})) {
            continue;
        }
        entry_for(k).max_tracks = rule.max_instances + course.extra_tracks_per_kind;
    }
}

void LandmarkServerNode::handle_set_focus(
    const std::shared_ptr<vortex_msgs::srv::SetMapFocus::Request> req,
    std::shared_ptr<vortex_msgs::srv::SetMapFocus::Response> res) {
    CourseModel& course = map_->course();
    if (!course.enabled()) {
        res->success = false;
        res->message = "no course layout loaded (course.enable is false)";
        return;
    }
    for (const auto& names : {req->commit, req->uncommit}) {
        for (const auto& name : names) {
            if (course.config().task(name) == nullptr) {
                res->success = false;
                res->message = "unknown task '" + name + "'";
                return;
            }
        }
    }
    if (const auto err = course.set_focus(req->tasks, req->lock_others)) {
        res->success = false;
        res->message = *err;
        return;
    }
    for (const auto& name : req->commit) {
        course.commit(name, true);
    }
    for (const auto& name : req->uncommit) {
        course.commit(name, false);
    }
    std::string focus;
    for (const auto& t : req->tasks) {
        focus += (focus.empty() ? "" : ", ") + t;
    }
    res->success = true;
    res->message = "focus: " + (focus.empty() ? std::string("all") : focus) +
                   (req->lock_others ? ", others locked" : "");
    spdlog::info("LandmarkServer: {}", res->message);
    publish_course_state();
}

void LandmarkServerNode::publish_course_state() {
    if (!course_state_pub_) {
        return;
    }
    const CourseModel& course = map_->course();
    vortex_msgs::msg::CourseState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;
    msg.course_frame_set = course_->status() != CourseFrameStatus::UNSET;
    msg.focus = course.focus();
    msg.lock_others = course.lock_others();
    const auto point = [](const Eigen::Vector3d& v) {
        geometry_msgs::msg::Point p;
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
        return p;
    };
    for (const auto& t : course.tasks()) {
        vortex_msgs::msg::CourseTask tm;
        tm.name = t.spec->name;
        tm.template_name = t.spec->template_name;
        tm.anchored = t.placed;
        tm.locked = course.locked(t);
        tm.committed = t.committed;
        tm.in_focus = course.in_focus(t);
        tm.variant = course.variant_name(t);
        tm.pose = vortex::utils::ros_conversions::to_pose_msg(
            vortex::utils::types::Pose::from_eigen(
                t.pose.translation(), Eigen::Quaterniond(t.pose.linear())));
        tm.region_radius = t.placed ? t.spec->part_radius_m : t.spec->region_radius_m;
        for (std::size_t i = 0; i < t.slots.size(); ++i) {
            const auto& s = t.slots[i];
            vortex_msgs::msg::CourseSlot sm;
            sm.name = s.name;
            const auto key = course.slot_class(t, i);
            sm.type.value = key.type;
            sm.subtype.value = key.subtype;
            sm.landmark_id = s.landmark_id;
            sm.live = s.track_id >= 0;
            sm.expected = point(course.slot_position(t, i));
            for (const auto& [c, n] : course.part_votes(t, i)) {
                if (c.first == key.type && c.second == key.subtype) {
                    sm.votes += n;
                } else {
                    sm.votes_other += n;
                }
            }
            tm.slots.push_back(sm);
        }
        msg.tasks.push_back(tm);
    }
    for (const auto& [reason, n] : drop_counts_) {
        msg.drop_reasons.push_back(reason);
        msg.drop_counts.push_back(n);
    }
    course_state_pub_->publish(msg);
}

}  // namespace vortex::mission
