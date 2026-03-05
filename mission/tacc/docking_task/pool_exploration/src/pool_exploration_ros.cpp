#include <pool_exploration/pool_exploration_ros.hpp>

#include <spdlog/spdlog.h>
#include <tf2/exceptions.hpp>
#include <tf2/time.h>

#include <rclcpp_components/register_node_macro.hpp>
// #include <vortex/utils/ros/qos_profiles.hpp>
#include <rclcpp/qos.hpp> //bytte med ovenfor?
#include "pool_exploration/pool_exploration.hpp"

namespace vortex::pool_exploration{

PoolExplorationNode::PoolExplorationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pool_exploration_node", options),
        size_x_(this->declare_parameter<double>("size_x", 10.0)),
        size_y_(this->declare_parameter<double>("size_y", 10.0)),
        resolution_(this->declare_parameter<double>("resolution", 0.1)),
        map_frame_(this->declare_parameter<std::string>("map_frame", "map")),
        map_(size_x_, size_y_, resolution_, map_frame_) {
    setup_publishers_and_subscribers();
} 

void PoolExplorationNode::setup_publishers_and_subscribers() {
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    
    pub_dt_ = std::chrono::milliseconds(
        this->declare_parameter<int>("publish_rate_ms", 200));

    // velge enu eller ned
    this->declare_parameter<bool>("enu_to_ned", false);

    const std::string line_sub_topic =
        this->declare_parameter<std::string>("line_sub_topic", "/line_detection/line_segments");

    const std::string map_pub_topic =
        this->declare_parameter<std::string>("map_pub_topic", "/map");
    // const std::string pose_sub_topic = 
    //     this->declare_parameter<std::string>("pose_sub_topic", "/pose"); // navn på denne????

    // Lager transformasjonen
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    const auto map_to_odom_tf = compute_map_odom_transform(); //const?
    tf_broadcaster_->sendTransform(map_to_odom_tf);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    const auto qos_sub = rclcpp::SensorDataQoS(); //standard for sensordata
    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)) //bare siste kart
        .reliable() //meldinger må leveres
        .transient_local(); //subscribers fr siste melding

    // subscriber
    auto sub_options = rclcpp::SubscriptionOptions(); //hva gjør denne, fjerne?
    line_sub_.subscribe(
        this,
        line_sub_topic,
        qos_sub.get_rmw_qos_profile(),
        sub_options); 
/*
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_sub_topic, qos_sensor_data,
        std::bind(&LOSGuidanceNode::pose_callback, this,
                  std::placeholders::_1));

*/
    // MessageFilter: slipper bare gjennom meldinger når TF til map_frame_ finnes
    line_filter_ = std::make_shared<tf2_ros::MessageFilter<vortex_msgs::msg::LineSegment2DArray>>(
        line_sub_, *tf_buffer_, map_frame_, 10,  // queue size
        this->get_node_logging_interface(),
        this->get_node_clock_interface());

    line_filter_->registerCallback(
        std::bind(&PoolExplorationNode::line_callback, this, std::placeholders::_1));

    // publisher
    //map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_topic, qos_pub); 
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_pub_topic, 
        map_qos);

    // service client
    waypoint_client_ = this->create_client<vortex_msgs::srv::SendWaypoints>("/send_waypoints"); //endre /send_waypoints navnet

    //timer 
    timer_ = this->create_wall_timer(pub_dt_, [this]() { this->timer_callback(); });
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


//FUNSKJON SOM TRANSFORMERER msg TIL LineSegmentene (Må dobbeltsjekke) 
std::vector<LineSegment> PoolExplorationNode::toMapSegments2D(
    const vortex_msgs::msg::LineSegment2DArray& msg,
    const Eigen::Matrix4f& T_map_src)
{
    //const Eigen::Matrix4f odom_to_map = map_to_odom_tf.inverse(); //Skal ikke inverse likevel, håndteres av lookupTransform
    
    //src er odom i dette tilfellet
    std::vector<LineSegment> segVector;
    segVector.reserve(msg.lines.size());

    for (const auto& line : msg.lines) {
        Eigen::Vector4f p0_src(line.p0.x, line.p0.y, 0.0f, 1.0f);
        Eigen::Vector4f p1_src(line.p1.x, line.p1.y, 0.0f, 1.0f);

        Eigen::Vector4f p0_map = T_map_src * p0_src;
        Eigen::Vector4f p1_map = T_map_src * p1_src;

        LineSegment seg;
        seg.p0 = {p0_map.x(), p0_map.y()};
        seg.p1 = {p1_map.x(), p1_map.y()};

        spdlog::info("[PoolExploration] Line received: ({:.2f}, {:.2f}) -> ({:.2f}, {:.2f})",
             seg.p0.x, seg.p0.y, seg.p1.x, seg.p1.y);


        segVector.push_back(seg);
    }
    return segVector;
}


void PoolExplorationNode::line_callback(
    const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg) {

    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(
            map_frame_, msg->header.frame_id, msg->header.stamp);
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("[PoolExploration] TF failed {} -> {}: {}",
                    msg->header.frame_id, map_frame_, ex.what());
        return;
    }

    const Eigen::Affine3d T = tf2::transformToEigen(tf_stamped.transform);
    const Eigen::Matrix4f T_map_src = T.matrix().cast<float>();

    //se nærmere på denne transform logikken
    auto segs = toMapSegments2D(*msg, T_map_src);
    // oppdaterer griddet direkte
    // bruker logikk fra pool_exploration.h
    map_.insertSegmentsMapFrame(segs);
    spdlog::info("[PoolExploration] {} line segments drawn in map", segs.size());

    // MÅ LEGGE INN DOCKING POSE ESTIMATION
        // ===== DOCKING ESTIMATE START =====

    // Midlertidige defaults til har ekte pose
    // MÅ ENDRE OG PLASSERE ANNET STED
    Eigen::Vector2f drone_pos = {0.0f, 0.0f};
    float drone_heading = 0.0f;

    float min_dist_ = 0.0f;        
    float max_dist_ = 50.0f;       
    float angle_threshold_ = 0.3f;  
    float min_angle_ = 0.7f;    
    float max_angle_  = 2.4f;
    float right_wall_offset_ = 0.5f;
    float far_wall_offset_ = 0.5f;
    
    auto corners = map_.findCorner(
        segs,
        drone_pos,
        drone_heading,
        min_dist_,          // float
        max_dist_,          // float
        angle_threshold_,   // float
        min_angle_,         // float (radians)
        max_angle_          // float (radians)
    );

    if (!corners.empty()) {
        CandidateCorner best = map_.selectBestCorner(corners, drone_pos);

        Eigen::Vector2f docking = map_.estimateDockingPosition(
            best,
            right_wall_offset_,   
            far_wall_offset_     
        );
        sendDockingWaypoint(docking);

        spdlog::info("[PoolExploration] Docking estimate (map): x={} y={}",
                    docking.x(), docking.y());
    } else {
        spdlog::info("[PoolExploration] No valid corners -> no docking estimate");
    }

    // ===== DOCKING ESTIMATE END =====

   // publish_grid();
}

void PoolExplorationNode::timer_callback() {
    publish_grid();
}

void PoolExplorationNode::publish_grid() {
    nav_msgs::msg::OccupancyGrid msg = map_.grid(); 

    msg.header.frame_id = map_frame_;
    msg.header.stamp = this->get_clock()->now();

    map_pub_->publish(msg);
}

void PoolExplorationNode::sendDockingWaypoint(const Eigen::Vector2f& docking_estimate)
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
    
    // hva skal orientation være??
    wp.pose.orientation.x = 0.0f;
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

RCLCPP_COMPONENTS_REGISTER_NODE(PoolExplorationNode)

}  // namespace vortex::pool_exploration
