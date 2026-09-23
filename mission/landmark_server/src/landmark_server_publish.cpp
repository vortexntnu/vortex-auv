#include <spdlog/spdlog.h>
#include <algorithm>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sstream>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include "landmark_server/landmark_server_ros.hpp"

namespace vortex::mission {

namespace {

/// Convert a parameter value from the overrides to YAML.
YAML::Node parameter_value_to_yaml(const rclcpp::ParameterValue& value) {
    YAML::Node node;
    switch (value.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            node = value.get<bool>();
            break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            node = value.get<int64_t>();
            break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            node = value.get<double>();
            break;
        case rclcpp::ParameterType::PARAMETER_STRING:
            node = value.get<std::string>();
            break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            for (const double d : value.get<std::vector<double>>()) {
                node.push_back(d);
            }
            break;
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            for (const auto i : value.get<std::vector<int64_t>>()) {
                node.push_back(i);
            }
            break;
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            for (const auto& str : value.get<std::vector<std::string>>()) {
                node.push_back(str);
            }
            break;
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            for (const bool b : value.get<std::vector<bool>>()) {
                node.push_back(b);
            }
            break;
        default:
            break;
    }
    return node;
}

/// Build a YAML tree from the dotted parameter names under the given roots.
YAML::Node overrides_to_yaml(
    const std::map<std::string, rclcpp::ParameterValue>& overrides,
    const std::vector<std::string>& roots) {
    YAML::Node root(YAML::NodeType::Map);
    for (const auto& [name, value] : overrides) {
        const auto first = name.substr(0, name.find('.'));
        if (std::find(roots.begin(), roots.end(), first) == roots.end()) {
            continue;
        }
        std::vector<std::string> parts;
        std::stringstream ss(name);
        for (std::string part; std::getline(ss, part, '.');) {
            parts.push_back(part);
        }
        YAML::Node node = root;
        for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
            if (!node[parts[i]]) {
                node[parts[i]] = YAML::Node(YAML::NodeType::Map);
            }
            node.reset(node[parts[i]]);
        }
        node[parts.back()] = parameter_value_to_yaml(value);
    }
    return root;
}

}  // namespace

void LandmarkServerNode::create_map() {
    const auto overrides =
        this->get_node_parameters_interface()->get_parameter_overrides();
    const YAML::Node tree = overrides_to_yaml(
        overrides, {"intake", "course_frame", "classes", "rules"});
    map_config_ = parse_map_config(tree);

    // Per-class track configs (track_config.<CLASS>...) on top of the default.
    const YAML::Node track_tree =
        overrides_to_yaml(overrides, {"track_config"});
    auto per_class = parse_per_class_track_config(
        track_tree["track_config"], track_manager_config_.default_class_config);
    if (!per_class.empty()) {
        track_manager_config_.per_class_configs = std::move(per_class);
        track_manager_ = std::make_unique<vortex::filtering::PoseTrackManager>(
            track_manager_config_);
    }

    map_ = std::make_unique<RetainedLandmarks>(map_config_);
    course_ = std::make_unique<CourseFrameTracker>(map_config_.course_frame);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto object_map_topic = this->declare_parameter<std::string>(
        "topics.object_map", "landmark_server/object_map");
    const auto live_tracks_topic = this->declare_parameter<std::string>(
        "topics.live_tracks", "landmark_server/live_tracks");
    const auto course_state_topic = this->declare_parameter<std::string>(
        "topics.course_frame_state", "landmark_server/course_frame_state");

    object_map_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            object_map_topic, rclcpp::QoS(10).reliable());
    live_tracks_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            live_tracks_topic, rclcpp::QoS(10).reliable());
    course_frame_state_pub_ =
        this->create_publisher<vortex_msgs::msg::CourseFrameState>(
            course_state_topic, rclcpp::QoS(1).reliable().transient_local());

    set_course_frame_srv_ =
        this->create_service<vortex_msgs::srv::SetCourseFrame>(
            "landmark_server/set_course_frame",
            [this](
                const std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Request>
                    req,
                std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Response>
                    res) { handle_set_course_frame(req, res); },
            rmw_qos_profile_services_default, timer_cb_group_);
    clear_srv_ = this->create_service<std_srvs::srv::Empty>(
        "landmark_server/clear",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
               std::shared_ptr<std_srvs::srv::Empty::Response> res) {
            handle_clear(req, res);
        },
        rmw_qos_profile_services_default, timer_cb_group_);

    publish_course_frame();
}

void LandmarkServerNode::handle_set_course_frame(
    const std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Request> req,
    std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Response> res) {
    const auto start_pose =
        vortex::utils::ros_conversions::ros_pose_to_pose(req->start_pose);
    const auto result =
        course_->set_coarse(start_pose, req->heading_offset_rad);
    res->success = result.success;
    res->message = result.message;
    if (result.success) {
        // A new frame invalidates what the old one let into the map.
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
    // otherwise put its landmark straight back.
    map_->clear();
    track_manager_ = std::make_unique<vortex::filtering::PoseTrackManager>(
        track_manager_config_);
    last_step_stamp_sec_.reset();
    spdlog::info("LandmarkServer: map cleared");
}

void LandmarkServerNode::reset_map() {
    map_->clear();
    course_->reset();
    publish_course_frame();
}

void LandmarkServerNode::update_map() {
    RetainedLandmarks::PositionFilter filter;
    if (course_->status() != CourseFrameStatus::UNSET) {
        filter = [this](const Eigen::Vector3d& p) {
            return course_->position_allowed(p);
        };
    }

    std::vector<vortex::filtering::Track> confirmed;
    for (const auto& t : track_manager_->get_tracks()) {
        if (t.confirmed) {
            confirmed.push_back(t);
        }
    }
    map_->update(confirmed, this->now().seconds(), filter);
}

vortex_msgs::msg::LandmarkTrack LandmarkServerNode::retained_to_msg(
    const RetainedLandmark& lm) const {
    vortex_msgs::msg::LandmarkTrack msg;
    const auto stamp = this->now();
    msg.header.stamp = stamp;
    msg.header.frame_id = target_frame_;

    msg.landmark.header = msg.header;
    msg.landmark.id = lm.id;
    msg.landmark.type.value = lm.key.type;
    msg.landmark.subtype.value = lm.key.subtype;
    msg.landmark.pose.pose = vortex::utils::ros_conversions::to_pose_msg(
        vortex::utils::types::Pose::from_eigen(lm.position, lm.orientation));

    // Covariance: state error order is (position, orientation).
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            msg.landmark.pose.covariance[r * 6 + c] = lm.covariance(r, c);
        }
    }
    if (!lm.has_orientation) {
        // Rotational variance >= the limit means no orientation.
        for (int i = 3; i < 6; ++i) {
            msg.landmark.pose.covariance[i * 6 + i] =
                map_config_.intake.no_orientation_rot_variance;
        }
    }

    msg.confirmed = true;
    msg.hits = lm.hits;
    msg.misses = lm.misses;
    msg.retained = lm.live_track_id < 0;
    msg.has_orientation = lm.has_orientation;
    msg.derived = lm.derived;
    msg.first_seen =
        rclcpp::Time(static_cast<int64_t>(lm.first_seen * 1e9), RCL_ROS_TIME);
    msg.last_measurement = rclcpp::Time(
        static_cast<int64_t>(lm.last_measurement * 1e9), RCL_ROS_TIME);
    msg.observations = lm.observations;
    return msg;
}

vortex_msgs::msg::LandmarkTrack LandmarkServerNode::live_track_to_msg(
    const vortex::filtering::Track& t) const {
    vortex_msgs::msg::LandmarkTrack msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;
    msg.landmark.header = msg.header;
    msg.landmark.id = t.id;
    msg.landmark.type.value = t.class_key.type;
    msg.landmark.subtype.value = t.class_key.subtype;
    msg.landmark.pose = track_to_pose_with_covariance(t);
    msg.confirmed = t.confirmed;
    msg.hits = t.hits();
    msg.misses = t.misses();
    msg.retained = false;
    msg.has_orientation = t.has_orientation;
    msg.derived = false;
    msg.last_measurement = msg.header.stamp;
    msg.first_seen = msg.header.stamp;
    return msg;
}

void LandmarkServerNode::publish_map() {
    vortex_msgs::msg::LandmarkTrackArray object_map;
    object_map.header.stamp = this->now();
    object_map.header.frame_id = target_frame_;
    for (const auto& lm : map_->landmarks()) {
        object_map.landmark_tracks.push_back(retained_to_msg(lm));
    }
    object_map_pub_->publish(object_map);

    vortex_msgs::msg::LandmarkTrackArray live;
    live.header = object_map.header;
    for (const auto& t : track_manager_->get_tracks()) {
        live.landmark_tracks.push_back(live_track_to_msg(t));
    }
    live_tracks_pub_->publish(live);
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
