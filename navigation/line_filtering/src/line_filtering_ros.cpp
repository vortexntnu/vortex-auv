#include <line_filtering/line_filtering_ros.hpp>

using std::placeholders::_1;

LineFilteringNode::LineFilteringNode() : Node("line_filtering_node")
{
    declare_parameter<double>("clutter_rate", 0.001);
    declare_parameter<double>("probability_of_detection", 0.7);
    declare_parameter<double>("probability_of_survival", 0.99);
    declare_parameter<double>("gate_threshold", 2.5);
    declare_parameter<double>("min_gate_threshold", 1.0);
    declare_parameter<double>("max_gate_threshold", 10.0);
    declare_parameter<double>("confirmation_threshold", 0.9);
    declare_parameter<double>("deletion_threshold", 0.1);
    declare_parameter<double>("initial_existence_probability", 0.4);
    declare_parameter<int>("update_interval_ms", 500);
    declare_parameter<double>("std_velocity", 0.2);
    declare_parameter<double>("std_sensor", 0.5);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");
    auto line_sub_topic = this->declare_parameter<std::string>("line_sub_topic", "/line/pose_array");
    auto camera_info_sub_topic = this->declare_parameter<std::string>("camera_info_sub_topic", "/cam_down/camera_info");
    auto dvl_depth_sub_topic = this->declare_parameter<std::string>("dvl_depth_sub_topic", "/dvl/depth");
    auto pose_array_pub_topic = this->declare_parameter<std::string>("pose_array_pub_topic", "/filtered_pose_array");
    auto odom_sub_topic = this->declare_parameter<std::string>("odom_sub_topic", "/orca/odom");
    debug_visualization_ = this->declare_parameter("debug_visualization", true);

    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
        );

    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    line_sub_.subscribe(this, line_sub_topic, qos.get_rmw_qos_profile());

    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseArray>>(
        line_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(), this->get_node_clock_interface());

    tf2_filter_->registerCallback(std::bind(&LineFilteringNode::line_callback, this, _1));

    // Subscriptions
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_sub_topic, qos_sensor_data, 
        std::bind(&LineFilteringNode::camera_info_callback, this, std::placeholders::_1));
    
    depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        dvl_depth_sub_topic, qos_sensor_data, 
        std::bind(&LineFilteringNode::depth_callback, this, std::placeholders::_1));
        
    pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        line_sub_topic, qos_sensor_data,
        std::bind(&LineFilteringNode::line_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_sub_topic, qos_sensor_data,
        std::bind(&LineFilteringNode::odom_callback, this, std::placeholders::_1));

    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(pose_array_pub_topic, qos_sensor_data);
    
    if (debug_visualization_)
    {
        point_1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_1", qos_sensor_data);
        point_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_2", qos_sensor_data);
        point_3_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_3", qos_sensor_data);
        point_4_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_4", qos_sensor_data);
    }

    depth_.data = -1.0;

    // Initialize track manager
    double std_velocity = get_parameter("std_velocity").as_double();
    double std_sensor = get_parameter("std_sensor").as_double();

    track_manager_ = TrackManager();
    track_manager_.set_dyn_model(std_velocity);
    track_manager_.set_sensor_model(std_sensor);

    // set timer
    int update_interval = get_parameter("update_interval_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&LineFilteringNode::timer_callback, this));

}

void LineFilteringNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    RCLCPP_INFO(this->get_logger(), "Camera Intrinsic Matrix initialized.");
    camera_info_received_ = true;
    camera_info_sub_.reset();
}

void LineFilteringNode::line_callback(const std::shared_ptr<const geometry_msgs::msg::PoseArray>& msg)
{
    if (depth_.data < 0.0) {
        RCLCPP_WARN(this->get_logger(), "Depth data not yet received.");
        return;
    }
    if (!camera_info_received_) {
        RCLCPP_WARN(this->get_logger(), "Camera info not yet received.");
        return;
    }

    double depth = depth_.data;

    try {
        geometry_msgs::msg::TransformStamped transform = tf2_buffer_->lookupTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero);

        size_t i = 0;
        size_t size = msg->poses.size();
        measurements_= Eigen::Array<double, 2, Eigen::Dynamic>(2, size);
        line_params_ = Eigen::Array<double, 2, Eigen::Dynamic>(2, size/2);

        for (const auto &pose : msg->poses) {

            double u = pose.position.x;
            double v = pose.position.y;

            // Transform 2D (u, v) to 3D (X, Y, Z) using the camera intrinsic matrix
            double X = (u - K_.at<double>(0, 2)) * depth / K_.at<double>(0, 0);
            double Y = (v - K_.at<double>(1, 2)) * depth / K_.at<double>(1, 1);

            tf2::Vector3 point(X, Y, depth);

            // Apply the transform to the position only
            tf2::Vector3 transformed = tf2::Transform(
                tf2::Quaternion(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
                tf2::Vector3(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
            ) * point;

            measurements_.col(i) << transformed.x(), transformed.y();
            if (i % 2 == 1) {
                line_params_.col((i-1)/2) = get_line_params(measurements_.block<2, 2>(0, i-1));
            }

            // Publish the transformed points for visualization
            if (debug_visualization_) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header = msg->header;
                pose_msg.pose.position.x = transformed.x();
                pose_msg.pose.position.y = transformed.y();
                pose_msg.pose.position.z = transformed.z();    

                switch (i) {
                    case 0: point_1_->publish(pose_msg); 
                        break;
                    case 1: point_2_->publish(pose_msg); 
                        break;
                    case 2: point_3_->publish(pose_msg); 
                        break;
                    case 3: point_4_->publish(pose_msg); 
                        break;
                    default: break;
                }
            }

            i++;
        }

    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

void LineFilteringNode::depth_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    depth_.data = msg->data;
}

void LineFilteringNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    orca_pose_ = msg->pose.pose;
}

Eigen::Vector2d LineFilteringNode::get_line_params(Eigen::Matrix<double, 2, 2> line_points)
{
    // Extract the two points (x1, y1) and (x2, y2)
    Eigen::Vector2d point1 = line_points.col(0);
    Eigen::Vector2d point2 = line_points.col(1);

    // Direction vector of the line
    Eigen::Vector2d direction = point2 - point1;

    // Normal vector to the line (perpendicular to the direction vector)
    Eigen::Vector2d normal(-direction.y(), direction.x());
    normal.normalize(); // Ensure the normal vector is of unit length

    // Compute the distance from the origin to the line
    // Use point1 for this calculation: distance = dot(normal, point1)
    double distance_to_origin = std::abs(normal.dot(point1));

    // Compute the angle of the normal vector with respect to the x-axis
    double angle = std::atan2(normal.y(), normal.x());

    // Return the parameters: [angle, distance]
    return Eigen::Vector2d(angle, distance_to_origin);
}

void LineFilteringNode::timer_callback()
{
    // get parameters
    int update_interval = get_parameter("update_interval_ms").as_int();
    double confirmation_threshold = get_parameter("confirmation_threshold").as_double();
    double gate_threshold = get_parameter("gate_threshold").as_double();
    double min_gate_threshold = get_parameter("min_gate_threshold").as_double();
    double max_gate_threshold = get_parameter("max_gate_threshold").as_double();
    double prob_of_detection = get_parameter("probability_of_detection").as_double();
    double prob_of_survival = get_parameter("probability_of_survival").as_double();
    double clutter_intensity = get_parameter("clutter_rate").as_double();
    double deletion_threshold = get_parameter("deletion_threshold").as_double();
    double initial_existence_probability = get_parameter("initial_existence_probability").as_double();

    // Update tracks
    track_manager_.updateTracks(measurements_, line_params_, update_interval, confirmation_threshold, gate_threshold, min_gate_threshold, max_gate_threshold, prob_of_detection, prob_of_survival,clutter_intensity, initial_existence_probability);

    measurements_.resize(2, 0);
    line_params_.resize(2, 0);

    // delete tracks
    track_manager_.deleteTracks(deletion_threshold);

    select_line();
}

void LineFilteringNode::select_line()
{
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = target_frame_;
    pose_array.header.stamp = this->now();
     for(const auto& track : track_manager_.getTracks())
    {   
        // Skips unconfirmed tracks
        if(track.confirmed == false){
            continue;
        }
        geometry_msgs::msg::Pose line_start;
        line_start.position.x = track.line_points(0, 0);
        line_start.position.y = track.line_points(1, 0);

        geometry_msgs::msg::Pose line_end;
        line_end.position.x = track.line_points(0, 1);
        line_end.position.y = track.line_points(1, 1);

        pose_array.poses.push_back(line_start);
        pose_array.poses.push_back(line_end);

        pose_array_pub_->publish(pose_array);
    }
}   

void LineFilteringNode::update_timer(int update_interval)
{
    timer_->cancel();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&LineFilteringNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Updated timer with %d ms update interval", update_interval);
}

void LineFilteringNode::update_dyn_model(double std_velocity)
{
    track_manager_.set_dyn_model(std_velocity);
    RCLCPP_INFO(this->get_logger(), "Updated dynamic model");
}

void LineFilteringNode::update_sensor_model(double std_measurement)
{
    track_manager_.set_sensor_model(std_measurement);
    RCLCPP_INFO(this->get_logger(), "Updated sensor model");
}