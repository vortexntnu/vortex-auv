#include "line_filtering/ros/line_filtering_ros.hpp"

#include <spdlog/spdlog.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>

namespace vortex::line_filtering {

LineFilteringNode::LineFilteringNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("line_filtering_node", options) {
    setup_publishers_and_subscribers();
    setup_track_manager();
}

void LineFilteringNode::setup_publishers_and_subscribers() {
    target_frame_ = this->declare_parameter<std::string>("target_frame");
    sonar_frame_ = this->declare_parameter<std::string>("sonar_frame");

    filter_dt_ = std::chrono::milliseconds(
        this->declare_parameter<int>("timer_rate_ms"));

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    const auto qos_sub = vortex::utils::qos_profiles::sensor_data_profile(10);
    const auto qos_pub = vortex::utils::qos_profiles::sensor_data_profile(1);

    const std::string line_sub_topic =
        this->declare_parameter<std::string>("line_sub_topic");
    const std::string sonar_info_sub_topic =
        this->declare_parameter<std::string>("sonar_info_sub_topic");

    create_line_subscription(line_sub_topic, qos_sub.get_rmw_qos_profile());

    sonar_info_sub_ = this->create_subscription<vortex_msgs::msg::SonarInfo>(
        sonar_info_sub_topic, qos_sub,
        [this](const vortex_msgs::msg::SonarInfo::ConstSharedPtr& msg) {
            this->sonar_info_callback(msg);
        });

    const std::string line_pub_topic =
        this->declare_parameter<std::string>("line_pub_topic");
    const std::string marker_pub_topic =
        this->declare_parameter<std::string>("marker_pub_topic");

    line_pub_ = this->create_publisher<vortex_msgs::msg::LineSegment3DArray>(
        line_pub_topic, qos_pub);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_pub_topic, qos_pub);

    pub_timer_ = this->create_wall_timer(filter_dt_,
                                         [this]() { this->timer_callback(); });

    debug_ = this->declare_parameter<bool>("debug.enable");
    if (debug_) {
        setup_debug_publishers();
    }
}

void LineFilteringNode::create_line_subscription(
    const std::string& topic_name,
    const rmw_qos_profile_t& qos_profile) {
    auto sub = std::make_shared<message_filters::Subscriber<LineMsgT>>(
        this, topic_name, qos_profile);

    auto filter = std::make_shared<tf2_ros::MessageFilter<LineMsgT>>(
        *sub, *tf2_buffer_, target_frame_, 10,
        this->get_node_logging_interface(), this->get_node_clock_interface());

    filter->registerCallback(
        [this](const typename LineMsgT::ConstSharedPtr msg) {
            this->line_segment_callback(msg);
        });

    line_sub_ = sub;
    tf_filter_ = filter;
}

void LineFilteringNode::setup_track_manager() {
    LineTrackManagerConfig config;

    config.existence.confirmation_threshold = this->declare_parameter<double>(
        "config.existence.confirmation_threshold");
    config.existence.deletion_threshold =
        this->declare_parameter<double>("config.existence.deletion_threshold");
    config.existence.initial_existence_probability =
        this->declare_parameter<double>(
            "config.existence.initial_existence_probability");

    config.default_config.min_rho_error =
        this->declare_parameter<double>("config.gate.min_rho_error");
    config.default_config.max_rho_error =
        this->declare_parameter<double>("config.gate.max_rho_error");
    config.default_config.min_phi_error =
        this->declare_parameter<double>("config.gate.min_phi_error");
    config.default_config.max_phi_error =
        this->declare_parameter<double>("config.gate.max_phi_error");

    config.default_config.dyn_std_dev =
        this->declare_parameter<double>("config.dyn_mod_std_dev");
    config.default_config.sens_std_dev =
        this->declare_parameter<double>("config.sens_mod_std_dev");

    config.default_config.init_rho_std =
        this->declare_parameter<double>("config.init_rho_std_dev");
    config.default_config.init_phi_std =
        this->declare_parameter<double>("config.init_phi_std_dev");

    config.default_config.mahalanobis_threshold =
        this->declare_parameter<double>("config.mahalanobis_gate_threshold");
    config.default_config.prob_of_detection =
        this->declare_parameter<double>("config.prob_of_detection");
    config.default_config.clutter_intensity =
        this->declare_parameter<double>("config.clutter_intensity");
    config.default_config.prob_of_survival =
        this->declare_parameter<double>("config.prob_of_survival");
    config.default_config.estimate_clutter =
        this->declare_parameter<bool>("config.estimate_clutter");

    track_manager_ = std::make_unique<LineTrackManager>(config);
}

void LineFilteringNode::sonar_info_callback(
    const vortex_msgs::msg::SonarInfo::ConstSharedPtr& msg) {
    latest_sonar_info_ = msg;
}

void LineFilteringNode::line_segment_callback(
    const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg) {
    spdlog::info("[LineFiltering] Received {} new line segments",
                 msg->lines.size());
    if (!latest_sonar_info_) {
        spdlog::warn(
            "[LineFiltering] No SonarInfo received yet. Ignoring "
            "line detections.");
        return;
    }

    // --- Pixel -> metric (sonar frame, z = 0) ---
    const double mpx = latest_sonar_info_->meters_per_pixel_x;
    const double mpy = latest_sonar_info_->meters_per_pixel_y;
    const double img_h = static_cast<double>(latest_sonar_info_->height);
    const double img_w = static_cast<double>(latest_sonar_info_->width);

    // Sonar image convention:
    //   pixel (0,0) = top-left, x right, y down.
    //   Sonar frame: x = forward (range axis), y = right (cross-range).
    //   We map pixel-y -> sonar-x (forward) with origin at bottom-centre,
    //   and pixel-x -> sonar-y (cross-range) with origin at image centre.

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf2_buffer_->lookupTransform(
            target_frame_, msg->header.frame_id, msg->header.stamp);
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("[LineFiltering] TF failed {} -> {}: {}",
                     msg->header.frame_id, target_frame_, ex.what());
        return;
    }

    const auto& t = tf_stamped.transform.translation;
    const auto& q = tf_stamped.transform.rotation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    Eigen::Matrix3d R = quat.toRotationMatrix();
    Eigen::Vector3d tvec(t.x, t.y, t.z);

    std::vector<LineMeasurement> new_meas;
    new_meas.reserve(msg->lines.size());

    for (const auto& seg : msg->lines) {
        // Convert pixel endpoints to metric sonar-frame xy (z=0).
        // sonar x = forward = (img_h - pixel_y) * mpy
        // sonar y = cross   = (img_w/2 - pixel_x) * mpx
        auto pixel_to_sonar = [&](double px, double py) -> Eigen::Vector3d {
            const double sx = (img_h - py) * mpy;        // forward
            const double sy = (img_w / 2.0 - px) * mpx;  // cross-range
            return Eigen::Vector3d{sx, sy, 0.0};
        };

        Eigen::Vector3d p0_sonar = pixel_to_sonar(seg.p0.x, seg.p0.y);
        Eigen::Vector3d p1_sonar = pixel_to_sonar(seg.p1.x, seg.p1.y);

        Eigen::Vector3d p0_odom = R * p0_sonar + tvec;
        Eigen::Vector3d p1_odom = R * p1_sonar + tvec;

        vortex::utils::types::LineSegment2D seg2d;
        seg2d.p0 = {p0_odom.x(), p0_odom.y()};
        seg2d.p1 = {p1_odom.x(), p1_odom.y()};

        if (debug_) {
            debug_odom_segments_.push_back(
                OdomSegment{.p0 = Eigen::Vector2d{p0_odom.x(), p0_odom.y()},
                            .p1 = Eigen::Vector2d{p1_odom.x(), p1_odom.y()}});
        }

        try {
            new_meas.push_back(LineMeasurement::from_segment(seg2d));
        } catch (const std::runtime_error& e) {
            spdlog::debug("[LineFiltering] Skipped degenerate segment: {}",
                          e.what());
        }
    }
    spdlog::info("[LineFiltering] Converted to {} line measurements after TF",
                 new_meas.size());

    measurements_.insert(measurements_.end(), new_meas.begin(), new_meas.end());
}

void LineFilteringNode::timer_callback() {
    spdlog::info("[LineFiltering] Timer tick: {} new measurements, {} tracks",
                 measurements_.size(), track_manager_->get_tracks().size());

    if (debug_) {
        publish_debug_meas();
    }

    track_manager_->step(measurements_,
                         std::chrono::duration<double>(filter_dt_).count());
    measurements_.clear();
    debug_odom_segments_.clear();

    spdlog::info("[LineFiltering] After step: {} tracks",
                 track_manager_->get_tracks().size());

    publish_tracks();
    publish_markers();

    if (debug_) {
        publish_debug_state();
    }
}

void LineFilteringNode::publish_tracks() {
    vortex_msgs::msg::LineSegment3DArray out;
    out.header.frame_id = target_frame_;
    out.header.stamp = this->get_clock()->now();

    constexpr double HALF_LEN = 5.0;

    for (const auto& track : track_manager_->get_tracks()) {
        if (!track.confirmed)
            continue;

        const Eigen::Vector2d& n = track.nominal.n;
        const double rho = track.nominal.rho;

        Eigen::Vector2d closest = rho * n;

        Eigen::Vector2d tangent(-n.y(), n.x());

        Eigen::Vector2d p0_2d = closest - HALF_LEN * tangent;
        Eigen::Vector2d p1_2d = closest + HALF_LEN * tangent;

        vortex_msgs::msg::LineSegment3D seg3d;
        seg3d.p0.x = p0_2d.x();
        seg3d.p0.y = p0_2d.y();
        seg3d.p0.z = 0.0;
        seg3d.p1.x = p1_2d.x();
        seg3d.p1.y = p1_2d.y();
        seg3d.p1.z = 0.0;

        out.lines.push_back(seg3d);
    }

    line_pub_->publish(out);
}

void LineFilteringNode::publish_markers() {
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto& track : track_manager_->get_tracks()) {
        if (!track.confirmed)
            continue;

        const Eigen::Vector2d& n = track.nominal.n;
        const double rho = track.nominal.rho;
        Eigen::Vector2d closest = rho * n;
        Eigen::Vector2d tangent(-n.y(), n.x());

        constexpr double HALF_LEN = 5.0;
        Eigen::Vector2d p0 = closest - HALF_LEN * tangent;
        Eigen::Vector2d p1 = closest + HALF_LEN * tangent;

        visualization_msgs::msg::Marker m;
        m.header.frame_id = target_frame_;
        m.header.stamp = this->get_clock()->now();
        m.ns = "line_tracks";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05;  // line width
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        m.color.a = static_cast<float>(track.existence_probability);

        geometry_msgs::msg::Point pt;
        pt.z = 0.0;

        pt.x = p0.x();
        pt.y = p0.y();
        m.points.push_back(pt);

        pt.x = p1.x();
        pt.y = p1.y();
        m.points.push_back(pt);

        marker_array.markers.push_back(m);
    }

    // Delete stale markers
    visualization_msgs::msg::Marker del;
    del.header.frame_id = target_frame_;
    del.header.stamp = this->get_clock()->now();
    del.ns = "line_tracks";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    // Publish DELETEALL first, then the new set
    visualization_msgs::msg::MarkerArray clear_array;
    clear_array.markers.push_back(del);
    marker_pub_->publish(clear_array);

    if (!marker_array.markers.empty()) {
        marker_pub_->publish(marker_array);
    }
}

void LineFilteringNode::setup_debug_publishers() {
    const std::string meas_topic =
        this->declare_parameter<std::string>("debug.meas_marker_topic");
    const std::string state_topic =
        this->declare_parameter<std::string>("debug.state_marker_topic");
    const auto qos = vortex::utils::qos_profiles::sensor_data_profile(10);

    debug_meas_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(meas_topic,
                                                                     qos);
    debug_state_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            state_topic, qos);
}

void LineFilteringNode::publish_debug_meas() {
    if (debug_odom_segments_.empty()) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    const auto now = this->get_clock()->now();

    // DELETEALL first so stale markers are removed
    {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = target_frame_;
        del.header.stamp = now;
        del.ns = "line_meas_raw";
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        visualization_msgs::msg::MarkerArray clear;
        clear.markers.push_back(del);
        debug_meas_marker_pub_->publish(clear);
    }

    int id = 0;
    for (const auto& seg : debug_odom_segments_) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = target_frame_;
        m.header.stamp = now;
        m.ns = "line_meas_raw";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.03;  // line width

        // Red for raw measurements
        m.color.r = 1.0f;
        m.color.g = 0.0f;
        m.color.b = 0.0f;
        m.color.a = 0.8f;

        geometry_msgs::msg::Point pt;
        pt.z = 0.0;

        pt.x = seg.p0.x();
        pt.y = seg.p0.y();
        m.points.push_back(pt);

        pt.x = seg.p1.x();
        pt.y = seg.p1.y();
        m.points.push_back(pt);

        marker_array.markers.push_back(m);
    }

    if (!marker_array.markers.empty()) {
        debug_meas_marker_pub_->publish(marker_array);
    }
}

void LineFilteringNode::publish_debug_state() {
    visualization_msgs::msg::MarkerArray marker_array;
    const auto now = this->get_clock()->now();

    // DELETEALL first
    {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = target_frame_;
        del.header.stamp = now;
        del.ns = "line_state_filtered";
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        visualization_msgs::msg::MarkerArray clear;
        clear.markers.push_back(del);
        debug_state_marker_pub_->publish(clear);
    }

    int id = 0;
    constexpr double HALF_LEN = 5.0;

    for (const auto& track : track_manager_->get_tracks()) {
        if (!track.confirmed)
            continue;

        const Eigen::Vector2d& n = track.nominal.n;
        const double rho = track.nominal.rho;
        Eigen::Vector2d closest = rho * n;
        Eigen::Vector2d tangent(-n.y(), n.x());
        Eigen::Vector2d p0 = closest - HALF_LEN * tangent;
        Eigen::Vector2d p1 = closest + HALF_LEN * tangent;

        visualization_msgs::msg::Marker m;
        m.header.frame_id = target_frame_;
        m.header.stamp = now;
        m.ns = "line_state_filtered";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05;  // line width

        // Green for filtered tracks
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        m.color.a = static_cast<float>(track.existence_probability);

        geometry_msgs::msg::Point pt;
        pt.z = 0.0;

        pt.x = p0.x();
        pt.y = p0.y();
        m.points.push_back(pt);

        pt.x = p1.x();
        pt.y = p1.y();
        m.points.push_back(pt);

        marker_array.markers.push_back(m);
    }

    if (!marker_array.markers.empty()) {
        debug_state_marker_pub_->publish(marker_array);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LineFilteringNode);

}  // namespace vortex::line_filtering
