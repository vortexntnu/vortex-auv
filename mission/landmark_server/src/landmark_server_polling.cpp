#include <spdlog/spdlog.h>
#include <algorithm>
#include <rclcpp_action/create_server.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include "landmark_server/landmark_server_ros.hpp"

// LandmarkPolling: wait until a confirmed live track of a class exists and
// return all of them. One goal at a time; a new goal aborts the old one.

namespace vortex::mission {

void LandmarkServerNode::create_polling_action_server() {
    polling_server_ = rclcpp_action::create_server<LandmarkPolling>(
        this,
        this->declare_parameter<std::string>("action_servers.landmark_polling"),
        [this](const rclcpp_action::GoalUUID&,
               std::shared_ptr<const LandmarkPolling::Goal>) {
            abort_polling_goal();
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<PollingGoalHandle>) {
            spdlog::info("LandmarkPolling: cancel requested");
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<PollingGoalHandle> goal) {
            spdlog::info("LandmarkPolling: waiting for type {} subtype {}",
                         goal->get_goal()->type.value,
                         goal->get_goal()->subtype.value);
            polling_goal_ = goal;
        });
}

void LandmarkServerNode::abort_polling_goal() {
    if (polling_goal_ && polling_goal_->is_active()) {
        polling_goal_->abort(std::make_shared<LandmarkPolling::Result>());
    }
    polling_goal_ = nullptr;
}

void LandmarkServerNode::serve_polling_goal() {
    if (!polling_goal_) {
        return;
    }
    if (polling_goal_->is_canceling()) {
        polling_goal_->canceled(std::make_shared<LandmarkPolling::Result>());
        polling_goal_ = nullptr;
        return;
    }
    if (!polling_goal_->is_active()) {
        return;
    }
    const auto goal = polling_goal_->get_goal();
    const uint16_t type = goal->type.value;
    const uint16_t subtype = goal->subtype.value;
    const auto& tracks = track_manager_->get_tracks();
    const bool found =
        subtype == 0
            ? std::any_of(tracks.begin(), tracks.end(),
                          [&](const auto& t) {
                              return t.confirmed && t.class_key.type == type;
                          })
            : track_manager_->has_track({type, subtype});
    if (!found) {
        return;
    }
    spdlog::info("LandmarkPolling: found type {} subtype {}", type, subtype);
    auto result = std::make_shared<LandmarkPolling::Result>();
    result->landmarks = tracks_to_landmark_msgs(type, subtype);
    polling_goal_->succeed(result);
}

vortex_msgs::msg::LandmarkArray LandmarkServerNode::tracks_to_landmark_msgs(
    uint16_t type,
    uint16_t subtype) const {
    std::vector<const vortex::filtering::Track*> tracks;
    if (subtype == 0) {
        for (const auto& t : track_manager_->get_tracks()) {
            if (t.confirmed && t.class_key.type == type) {
                tracks.push_back(&t);
            }
        }
    } else {
        tracks = track_manager_->get_tracks_by_type({type, subtype});
    }
    vortex_msgs::msg::LandmarkArray out;
    out.header.stamp = this->now();
    out.header.frame_id = target_frame_;
    out.landmarks.reserve(tracks.size());
    for (const auto* t : tracks) {
        vortex_msgs::msg::Landmark lm;
        lm.type.value = t->class_key.type;
        lm.subtype.value = t->class_key.subtype;
        lm.id = t->id;
        lm.pose.pose = vortex::utils::ros_conversions::to_pose_msg(t->to_pose());
        out.landmarks.push_back(std::move(lm));
    }
    return out;
}

}  // namespace vortex::mission
