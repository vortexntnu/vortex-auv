#include <spdlog/spdlog.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include "landmark_server/landmark_server_ros.hpp"
#include "landmark_server/map_rules.hpp"

namespace vortex::mission {

void LandmarkServerNode::create_outputs() {
    using vortex_msgs::srv::SetCourseFrame;
    using vortex_msgs::srv::SetMapFocus;
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    object_map_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            this->declare_parameter<std::string>("topics.object_map",
                                                 "landmark_server/object_map"),
            rclcpp::QoS(10).reliable());
    course_frame_state_pub_ =
        this->create_publisher<vortex_msgs::msg::CourseFrameState>(
            this->declare_parameter<std::string>(
                "topics.course_frame_state",
                "landmark_server/course_frame_state"),
            rclcpp::QoS(1).reliable().transient_local());
    course_state_pub_ = this->create_publisher<vortex_msgs::msg::CourseState>(
        this->declare_parameter<std::string>("topics.course_state",
                                             "landmark_server/course_state"),
        rclcpp::QoS(1).reliable().transient_local());

    // The services run in the timer's callback group: they never see the
    // map half-way through a tick.
    set_course_frame_srv_ =
        this->create_service<SetCourseFrame>(
            "landmark_server/set_course_frame",
            [this](const std::shared_ptr<SetCourseFrame::Request> req,
                   std::shared_ptr<SetCourseFrame::Response> res) {
                handle_set_course_frame(req, res);
            },
            rmw_qos_profile_services_default, timer_cb_group_);
    clear_srv_ = this->create_service<std_srvs::srv::Empty>(
        "landmark_server/clear",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
               std::shared_ptr<std_srvs::srv::Empty::Response> res) {
            handle_clear(req, res);
        },
        rmw_qos_profile_services_default, timer_cb_group_);
    set_focus_srv_ = this->create_service<SetMapFocus>(
        "landmark_server/set_focus",
        [this](const std::shared_ptr<SetMapFocus::Request> req,
               std::shared_ptr<SetMapFocus::Response> res) {
            handle_set_focus(req, res);
        },
        rmw_qos_profile_services_default, timer_cb_group_);

    publish_course_frame();
}

void LandmarkServerNode::handle_set_course_frame(
    const std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Request> req,
    std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Response> res) {
    const auto result = course_->set_coarse(
        vortex::utils::ros_conversions::ros_pose_to_pose(req->start_pose),
        req->heading_offset_rad);
    res->success = result.success;
    res->message = result.message;
    if (result.success) {
        spdlog::info("LandmarkServer: course frame set, through_yaw={:.3f} rad",
                     course_->through_yaw());
    } else {
        spdlog::warn("LandmarkServer: set_course_frame rejected: {}",
                     result.message);
    }
    res->state = course_frame_state_msg();
    publish_course_frame();
}

void LandmarkServerNode::handle_clear(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
    // The map and the live tracks: a track that is still confirmed would
    // otherwise put its landmark straight back. The course frame stays.
    map_->clear();
    drop_counts_.clear();
    clear_graph();
    track_manager_ = std::make_unique<vortex::filtering::PoseTrackManager>(
        track_manager_config_);
    filter_time_sec_.reset();
    spdlog::info("LandmarkServer: map cleared");
}

void LandmarkServerNode::reset_map() {
    map_->clear();
    drop_counts_.clear();
    clear_graph();
    course_->reset();
    publish_course_frame();
}

void LandmarkServerNode::update_map() {
    RetainedLandmarks::PositionFilter filter;
    if (course_->status() != CourseFrameStatus::UNSET &&
        map_->course().enabled()) {
        const CourseGeometry geo = course_geometry();
        filter = [this, geo](const Eigen::Vector3d& p) {
            return map_->course().lane_allows(p, geo);
        };
    }

    std::vector<vortex::filtering::Track> confirmed;
    for (const auto& t : track_manager_->get_tracks()) {
        if (t.confirmed) {
            confirmed.push_back(t);
        }
    }
    // The classes the detector reported for each track this tick (the
    // course model votes with them), and the course frame.
    RetainedLandmarks::CourseInput course_input;
    for (const auto& a : tick_associations_) {
        const auto& c = a.measurement.reported_class;
        ++course_input.votes[a.track_id][{c.type, c.subtype}];
    }
    course_input.geometry = course_geometry();
    const double now = this->now().seconds();
    map_->update(confirmed, now, filter, course_input);
    // Smoothed positions replace the tracker's before the rules use them.
    update_graph();

    // The rules need the vehicle position; without odometry they wait.
    std::optional<geometry_msgs::msg::Point> vehicle;
    {
        std::lock_guard<std::mutex> lock(odom_mtx_);
        vehicle = last_odom_position_;
    }
    if (!vehicle) {
        return;
    }
    apply_map_rules(*map_, map_config_.map_rules,
                    Eigen::Vector3d(vehicle->x, vehicle->y, vehicle->z), now,
                    course_.get());
    if (course_->take_deviation_warning()) {
        spdlog::warn(
            "LandmarkServer: the gate direction differs {:.1f} deg from the "
            "start value of the course frame; the gate wins",
            course_->start_vs_gate_deviation_deg());
    }
}

vortex_msgs::msg::LandmarkTrack LandmarkServerNode::retained_to_msg(
    const RetainedLandmark& lm) const {
    vortex_msgs::msg::LandmarkTrack msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;
    msg.landmark.header = msg.header;
    msg.landmark.id = lm.id;
    msg.landmark.type.value = lm.key.type;
    msg.landmark.subtype.value = lm.key.subtype;
    msg.landmark.pose.pose = vortex::utils::ros_conversions::to_pose_msg(
        vortex::utils::types::Pose::from_eigen(lm.position, lm.orientation));
    // Covariance order (position, orientation), as the state.
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            msg.landmark.pose.covariance[r * 6 + c] = lm.covariance(r, c);
        }
    }
    if (!lm.has_orientation) {
        // A rotation variance at the limit means "no orientation".
        for (int i = 3; i < 6; ++i) {
            msg.landmark.pose.covariance[i * 6 + i] =
                map_config_.intake.no_orientation_rot_variance;
        }
    }
    msg.confirmed = true;
    msg.hits = lm.hits;
    msg.misses = lm.misses;
    msg.retained = !lm.is_live();
    msg.has_orientation = lm.has_orientation;
    msg.derived = lm.derived;
    msg.first_seen =
        rclcpp::Time(static_cast<int64_t>(lm.first_seen * 1e9), RCL_ROS_TIME);
    msg.last_measurement = rclcpp::Time(
        static_cast<int64_t>(lm.last_measurement * 1e9), RCL_ROS_TIME);
    msg.observations = lm.observations;
    return msg;
}

void LandmarkServerNode::publish_map() {
    vortex_msgs::msg::LandmarkTrackArray object_map;
    object_map.header.stamp = this->now();
    object_map.header.frame_id = target_frame_;
    for (const auto& lm : map_->landmarks()) {
        if (lm.absorbed_by >= 0) {
            continue;  // described better by another landmark
        }
        object_map.landmark_tracks.push_back(retained_to_msg(lm));
    }
    object_map_pub_->publish(object_map);
}

vortex_msgs::msg::CourseFrameState LandmarkServerNode::course_frame_state_msg()
    const {
    using State = vortex_msgs::msg::CourseFrameState;
    State msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;
    switch (course_->status()) {
        case CourseFrameStatus::COARSE:
            msg.state = State::COARSE;
            break;
        case CourseFrameStatus::GATE_LOCKED:
            msg.state = State::GATE_LOCKED;
            break;
        default:
            msg.state = State::UNSET;
            break;
    }
    msg.through_yaw = course_->through_yaw();
    msg.yaw_std = course_->yaw_std();
    msg.consistent_estimates = course_->consistent_estimates();
    msg.start_vs_gate_deviation_deg = course_->start_vs_gate_deviation_deg();
    return msg;
}

void LandmarkServerNode::publish_course_frame() {
    course_frame_state_pub_->publish(course_frame_state_msg());

    // No TF while the frame does not exist, so nobody computes in it.
    if (!map_config_.course_frame.publish_tf ||
        course_->status() == CourseFrameStatus::UNSET) {
        return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = target_frame_;
    tf.child_frame_id = map_config_.course_frame.frame_id;
    tf.transform.translation.x = course_->origin().x();
    tf.transform.translation.y = course_->origin().y();
    tf.transform.translation.z = 0.0;
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(course_->through_yaw(), Eigen::Vector3d::UnitZ()));
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
}

}  // namespace vortex::mission
