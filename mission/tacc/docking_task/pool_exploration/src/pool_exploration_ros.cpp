#include <memory>
#include <pool_exploration/pool_exploration_ros.hpp>

#include <spdlog/spdlog.h>
#include <rclcpp/node_options.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.h>

#include <rclcpp_components/register_node_macro.hpp>
// #include <vortex/utils/ros/qos_profiles.hpp>
#include <rclcpp/qos.hpp> //bytte med ovenfor?
#include <vortex/utils/types.hpp>
#include "pool_exploration/pool_exploration.hpp"

namespace vortex::pool_exploration{

PoolExplorationNode::PoolExplorationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pool_exploration_node", options) {
        setup_publishers_and_subscribers();
        setup_planner();
    } 

void PoolExplorationNode::setup_publishers_and_subscribers() {
    //map_frame_ = this->declare_parameter<std::string>("map_frame", "map"); //allerede definert
    odom_frame_ = this->declare_parameter<std::string>("odom_frame");
    base_frame_ = this->declare_parameter<std::string>("base_frame");
    sonar_frame_ = this->declare_parameter<std::string>("sonar_frame");

    //pub_dt_ = std::chrono::milliseconds(
    //    this->declare_parameter<int>("publish_rate_ms"));

    //this->declare_parameter<bool>("enu_to_ned", false);

    const std::string line_sub_topic =
        this->declare_parameter<std::string>("line_sub_topic");
    //const std::string map_pub_topic =
    //    this->declare_parameter<std::string>("map_pub_topic", "/map");
    const std::string pose_sub_topic = 
         this->declare_parameter<std::string>("pose_sub_topic"); // ta inn drone pos

    const std::string sonar_info_sub_topic =
        this->declare_parameter<std::string>("sonar_info_sub_topic");

    const std::string debug_topic = 
        this->declare_parameter<std::string>("debug_topic");

    //tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    //const auto map_to_odom_tf = compute_map_odom_transform(); //const?
    //tf_broadcaster_->sendTransform(map_to_odom_tf);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos_sub = rclcpp::SensorDataQoS(); //standard for sensordata, ta inn vortex sin i stedet??
    //const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)) //bare siste kart
    //    .reliable() //meldinger må leveres
    //    .transient_local(); //subscribers fra siste melding

    auto sub_options = rclcpp::SubscriptionOptions(); //hva gjør denne, fjerne?
    line_sub_.subscribe(
        this,
        line_sub_topic,
        qos_sub.get_rmw_qos_profile(),
        sub_options); 

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_sub_topic, qos_sub,
        std::bind(&PoolExplorationNode::pose_callback, this,
                  std::placeholders::_1));

    sonar_info_sub_ = this->create_subscription<vortex_msgs::msg::SonarInfo>(
        sonar_info_sub_topic, qos_sub,
        [this](const vortex_msgs::msg::SonarInfo::ConstSharedPtr& msg) {
            this->sonar_info_callback(msg);
        });

    line_filter_ = std::make_shared<tf2_ros::MessageFilter<vortex_msgs::msg::LineSegment2DArray>>(
        line_sub_, *tf_buffer_, odom_frame_, 10, // Har endra til odom frame fra map frame
        this->get_node_logging_interface(),
        this->get_node_clock_interface());

    line_filter_->registerCallback(
        std::bind(&PoolExplorationNode::line_callback, this, std::placeholders::_1));

    //map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    //    map_pub_topic, 
    //    map_qos);

    waypoint_client_ = this->create_client<vortex_msgs::srv::SendWaypoints>("/send_waypoints"); //endre /send_waypoints navnet

    //timer_ = this->create_wall_timer(pub_dt_, [this]() { this->timer_callback(); });

    docking_marker_pub_ = //til testing
        this->create_publisher<visualization_msgs::msg::Marker>(
        debug_topic, 10);
}

void PoolExplorationNode::setup_planner() { //change name?
    PoolExplorationPlannerConfig config{};

    config.min_wall_distance_m = this->declare_parameter<double>("min_wall_distance_m");
    config.max_wall_distance_m = this->declare_parameter<double>("max_wall_distance_m");
    config.far_wall_heading_angle_threshold = this->declare_parameter<double>("far_wall_heading_angle_threshold");
    config.min_corner_angle_rad = this->declare_parameter<double>("min_corner_angle_rad");
    config.max_corner_angle_rad = this->declare_parameter<double>("max_corner_angle_rad");
    config.right_dist = this->declare_parameter<double>("right_dist");
    //config.left_dist = this->declare_parameter<double>("left_dist");
    config.right_wall_offset_m = this->declare_parameter<double>("right_wall_offset_m");
    config.far_wall_offset_m = this->declare_parameter<double>("far_wall_offset_m");

    planner_ = std::make_unique<PoolExplorationPlanner>(config);
}

void PoolExplorationNode::sonar_info_callback(
    const vortex_msgs::msg::SonarInfo::ConstSharedPtr& msg) {
    latest_sonar_info_ = msg;
}

std::vector<vortex::utils::types::LineSegment2D> PoolExplorationNode::transform_segments_2d( //FUNSKJON SOM TRANSFORMERER msg TIL LineSegmentene (Må dobbeltsjekke) 
    const vortex_msgs::msg::LineSegment2DArray& msg,
    const Eigen::Matrix4f& T_target_src) //target er map/odom osv
{
    std::vector<vortex::utils::types::LineSegment2D> segments;
    segments.reserve(msg.lines.size());
    
    if (!latest_sonar_info_) {
        spdlog::warn("[PoolExploration] No sonar image info available yet");
        return segments;
    }

    vortex::utils::types::SonarInfo sonar_info;
    sonar_info.meters_per_pixel_x = latest_sonar_info_->meters_per_pixel_x;
    sonar_info.meters_per_pixel_y = latest_sonar_info_->meters_per_pixel_y;
    sonar_info.image_width  = latest_sonar_info_->width;
    sonar_info.image_height = latest_sonar_info_->height;

    for (const auto& line : msg.lines) {
        const auto p0 = sonar_info.pixel_index_to_sonar_metric(line.p0.x, line.p0.y);
        const auto p1 = sonar_info.pixel_index_to_sonar_metric(line.p1.x, line.p1.y);

        const Eigen::Vector4f p0_sonar(
            static_cast<float>(p0.y),
            static_cast<float>(p0.x),
            0.0f,
            1.0f
        );
        const Eigen::Vector4f p1_sonar(
            static_cast<float>(p1.y),
            static_cast<float>(p1.x),
            0.0f,
            1.0f
        );

        const Eigen::Vector4f p0_target = T_target_src * p0_sonar;
        const Eigen::Vector4f p1_target = T_target_src * p1_sonar;

        vortex::utils::types::LineSegment2D seg;
        seg.p0 = {p0_target.x(), p0_target.y()};
        seg.p1 = {p1_target.x(), p1_target.y()};

        spdlog::info("[PoolExploration] Line received: ({:.2f}, {:.2f}) -> ({:.2f}, {:.2f})", // til debugging
             seg.p0.x, seg.p0.y, seg.p1.x, seg.p1.y);

        segments.push_back(seg);
    }
    return segments;
}

void PoolExplorationNode::line_callback(
    const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg) {
    
    estimate_and_send_docking_waypoint(*msg);
    //drawSegmentsInMapFrame(*msg);
}

void PoolExplorationNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
    drone_state_.x = msg->pose.pose.position.x;
    drone_state_.y = msg->pose.pose.position.y;
    drone_state_.z = msg->pose.pose.position.z;

    Eigen::Quaterniond q;
    tf2::fromMsg(msg->pose.pose.orientation, q);

    Eigen::Vector3d rpy = vortex::utils::math::quat_to_euler(q);

    drone_state_.yaw = rpy.z();
    }

// GJØRE I base_frame ELLER odom??
void PoolExplorationNode::estimate_and_send_docking_waypoint(
    const vortex_msgs::msg::LineSegment2DArray& msg) {
    geometry_msgs::msg::TransformStamped tf_stamped;

    try {
        tf_stamped = tf_buffer_->lookupTransform(
            odom_frame_, msg.header.frame_id, msg.header.stamp);
    //        base_frame_, msg.header.frame_id, msg.header.stamp);
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("[PoolExploration] TF failed {} -> {}: {}",
                     msg.header.frame_id, odom_frame_, ex.what());
    //                 msg.header.frame_id, base_frame_,ex.what());
        return;
    }

    const Eigen::Affine3d T = tf2::transformToEigen(tf_stamped.transform);
    const Eigen::Matrix4f T_odom_src = T.matrix().cast<float>();

    auto segs = transform_segments_2d(msg, T_odom_src); //NB ENDRE NAVN <3

    Eigen::Vector2f drone_pos = {drone_state_.x,drone_state_.y};     // Teste om fungerer?
    float drone_heading = drone_state_.yaw;

    auto corners = planner_->find_corner_estimates(
        segs,
        drone_pos,
        drone_heading
    );

    if (corners.empty()) {
        spdlog::info("[PoolExploration] No valid corners -> no docking estimate");
        return;
    }

    CornerEstimate best_corner = planner_->select_best_corner(corners, drone_pos);

    Eigen::Vector2f docking = planner_->estimate_docking_position(best_corner, drone_pos);

    publish_docking_marker(docking); //til testing
    send_docking_waypoint(docking);

    spdlog::info("[PoolExploration] Docking estimate (odom): x={} y={}",
                 docking.x(), docking.y());
}

void PoolExplorationNode::send_docking_waypoint(const Eigen::Vector2f& docking_estimate)
{
    if (waypoint_sent_) {
        return;
    }

    if (!waypoint_client_->service_is_ready()) {
        spdlog::warn("Waypoint service not available");
        return;
    }

    auto request =
        std::make_shared<vortex_msgs::srv::SendWaypoints::Request>();

    vortex_msgs::msg::Waypoint wp;

    wp.pose.position.x = docking_estimate.x();
    wp.pose.position.y = docking_estimate.y();
    wp.pose.position.z = 0.0f; //Hva skal denne være
    
    wp.pose.orientation.x = 0.0f;     // hva skal orientation være??
    wp.pose.orientation.y = 0.0f;
    wp.pose.orientation.z = 0.0f;
    wp.pose.orientation.w = 1.0f;

    request->waypoints.push_back(wp);

    request->switching_threshold = 0.5;
    request->overwrite_prior_waypoints = true;
    request->take_priority = true;

    waypoint_client_->async_send_request(request);

    waypoint_sent_ = true;

    spdlog::info("Docking waypoint sent to WaypointManager");
}

void PoolExplorationNode::publish_docking_marker(const Eigen::Vector2f& docking) //til testing
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = odom_frame_;
    marker.header.stamp = this->now();

    marker.ns = "docking_debug";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = docking.x();
    marker.pose.position.y = docking.y();
    marker.pose.position.z = 0.0;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    docking_marker_pub_->publish(marker);
}

// Grid logikk
# if 0
//Konstruktøren
PoolExplorationNode::PoolExplorationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pool_exploration_node", options),
        size_x_(this->declare_parameter<double>("size_x")),
        size_y_(this->declare_parameter<double>("size_y")),
        resolution_(this->declare_parameter<double>("resolution")),
        map_(size_x_, size_y_, resolution_, map_frame_) {
    setup_publishers_and_subscribers();
}

geometry_msgs::msg::TransformStamped PoolExplorationNode::compute_map_odom_transform() {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->get_clock()->now();
    map_to_odom.header.frame_id = map_frame_;
    map_to_odom.child_frame_id = odom_frame_;

    // dette blir i midten av kartet
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;

    if (this->get_parameter("enu_to_ned").as_bool()) {
        tf2::Quaternion q1;
        q1.setRPY(M_PI, 0.0, M_PI_2);
        map_to_odom.transform.rotation.w = q1.w();
        map_to_odom.transform.rotation.x = q1.x();
        map_to_odom.transform.rotation.y = q1.y();
        map_to_odom.transform.rotation.z = q1.z();
    } else {
        //Kjøres dersom vi er i ENU frame
        tf2::Quaternion q2;
        q2.setRPY(0.0, 0.0, 0.0);
        map_to_odom.transform.rotation.w = q2.w();
        map_to_odom.transform.rotation.x = q2.x();
        map_to_odom.transform.rotation.y = q2.y();
        map_to_odom.transform.rotation.z = q2.z();
    }

    return map_to_odom;
}

// VENTE MED DENNE LOGIKK TIL SENERE
void PoolExplorationNode::draw_segments_in_map_frame(
    const vortex_msgs::msg::LineSegment2DArray& msg)
{
    geometry_msgs::msg::TransformStamped tf_stamped;

    try {
        tf_stamped = tf_buffer_->lookupTransform(
            map_frame_, msg.header.frame_id, msg.header.stamp);
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("[PoolExploration] TF failed {} -> {}: {}",
                     msg.header.frame_id, map_frame_, ex.what());
        return;
    }

    const Eigen::Affine3d T = tf2::transformToEigen(tf_stamped.transform);
    const Eigen::Matrix4f T_map_src = T.matrix().cast<float>();

    //se nærmere på denne transform logikken
    auto segs = transform_segments_2d(msg, T_map_src);
    // bruker logikk fra pool_exploration.h

    map_.insert_line_in_grid(segs);

    spdlog::info("[PoolExploration] {} line segments drawn in map", segs.size());
}
// LOGIKK FOR GRIDDET
void PoolExplorationNode::publish_grid() {
    nav_msgs::msg::OccupancyGrid msg = map_.grid(); 

    msg.header.frame_id = map_frame_;
    msg.header.stamp = this->get_clock()->now();

    map_pub_->publish(msg);
}

void PoolExplorationNode::timer_callback() {
    publish_grid();
}
#endif

RCLCPP_COMPONENTS_REGISTER_NODE(PoolExplorationNode)

}  // namespace vortex::pool_exploration
