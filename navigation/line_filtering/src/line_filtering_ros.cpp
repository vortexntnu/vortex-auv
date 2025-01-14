#include "line_filtering/line_filtering_ros.hpp"
#include "line_filtering/track_manager.hpp"

Camera3DPointsNode::Camera3DPointsNode() : Node("camera_3d_points_node")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Subscriptions
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/cam_down/camera_info", qos_sensor_data, 
        std::bind(&Camera3DPointsNode::cameraInfoCallback, this, std::placeholders::_1));
    
    depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/dvl/depth", qos_sensor_data, 
        std::bind(&Camera3DPointsNode::depthCallback, this, std::placeholders::_1));
        
    pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/line/pose_array", qos_sensor_data,
        std::bind(&Camera3DPointsNode::poseArrayCallback, this, std::placeholders::_1));

    point_1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_1", qos_sensor_data);
    point_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_2", qos_sensor_data);
    point_3_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_3", qos_sensor_data);
    point_4_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_4", qos_sensor_data);


    
    depth_.data = -1.0;

    // set timer
    int update_interval = get_parameter("update_interval_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&line_filtering_node::timer_callback, this));


    //publisher
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pose_array", qos_sensor_data);
}

void Camera3DPointsNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    RCLCPP_INFO(this->get_logger(), "Camera Intrinsic Matrix initialized.");
    camera_info_received_ = true;
    camera_info_sub_.reset();
}



void Camera3DPointsNode::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
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

    RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());

    // Create a new PoseArray to publish
    geometry_msgs::msg::PoseArray updated_pose_array;
    updated_pose_array.header = msg->header;  // Use the same header for consistency

    // Iterate through the incoming poses and compute 3D points
    size_t i = 0;
    for (const auto &pose : msg->poses) {
        geometry_msgs::msg::Pose updated_pose;

        // Extract x and y from the incoming PoseArray
        double u = pose.position.x;
        double v = pose.position.y;

        // Transform 2D (u, v) to 3D (X, Y, Z) using the camera intrinsic matrix
        double X = (u - K_.at<double>(0, 2)) * depth / K_.at<double>(0, 0);
        double Y = (v - K_.at<double>(1, 2)) * depth / K_.at<double>(1, 1);

        // Create the updated pose
        updated_pose.position.x = X;
        updated_pose.position.y = Y;
        updated_pose.position.z = depth;  // Use the depth value from DVL
        updated_pose.orientation = pose.orientation;  // Retain the orientation from the incoming pose

        // Add the updated pose to the PoseArray
        updated_pose_array.poses.push_back(updated_pose);

        // Publish individual PoseStamped messages for specific points (optional)
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose = updated_pose;        


        switch (i) {
            case 0: point_1_->publish(pose_msg); 
                linePointsArray_.poses[0] = point_1_; //Store the pose in the pose array
                break;
            case 1: point_2_->publish(pose_msg); 
                linePointsArray_.poses[1] = point_2_; //Store the pose in the pose array
                break;
            case 2: point_3_->publish(pose_msg); 
                linePointsArray_.poses[2] = point_3_; //Store the pose in the pose array
                break;
            case 3: point_4_->publish(pose_msg); 
                linePointsArray_.poses[3] = point_4_; //Store the pose in the pose array
                break;
            default: break;
        }

        RCLCPP_INFO(this->get_logger(), "Point %zu: 2D (%f, %f) -> 3D (%f, %f, %f)", 
                    i + 1, u, v, X, Y, depth);

        i++;
    }

    // Publish the updated PoseArray
    pose_array_pub_->publish(updated_pose_array);

    RCLCPP_INFO(this->get_logger(), "Published updated PoseArray with %zu poses", updated_pose_array.poses.size());
}

void Camera3DPointsNode::depthCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    depth_.data = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received depth: %f", msg->data);
}

Camera3DPointsNode::LineGrouper Camera3DPointsNode::lines_grouped(geometry_msgs::msg::Point point1_, 
                                                                  geometry_msgs::msg::Point point2_, 
                                                                  geometry_msgs::msg::Point point3_, 
                                                                  geometry_msgs::msg::Point point4_) {
    Camera3DPointsNode::LineGrouper lines_grouped_temp;
    // retrieve member variables on timer callback
    lines_grouped_temp.line1_.start = point1_;
    lines_grouped_temp.line1_.end = point2_;
    lines_grouped_temp.line2_.start = point3_;
    lines_grouped_temp.line2_.end = point4_;

    return lines_grouped_temp;
}


//Selecting line based on Y values
Camera3DPointsNode::LineGrouper Camera3DPointsNode::line_selector(Camera3DPointsNode::PointGrouper& line1, 
                                                                  Camera3DPointsNode::PointGrouper& line2)
{
    Camera3DPointsNode::LineGrouper selected_line;

    if (line1.start.y > line2.end.y) {
        selected_line.line1_ = line1;
    } else {
        selected_line.line2_ = line2;
    }

    return selected_line;
}

//get current pose from DVL and orientation for orca, to test if we travel too far 

void Camera3DPointsNode::timer_callback()
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

    // Update tracks
    track_manager_.updateTracks(measurements_, update_interval, confirmation_threshold, gate_threshold, min_gate_threshold, max_gate_threshold, prob_of_detection, prob_of_survival,clutter_intensity);

    measurements_.clear();

    // Publish tracks to landmark server
    publish_landmarks(deletion_threshold);

    // Publish visualization parameters
    if (get_parameter("publish_visualization").as_bool())
    {
        publish_visualization_parameters(gate_threshold, min_gate_threshold, max_gate_threshold);
    }

    // delete tracks
    track_manager_.deleteTracks(deletion_threshold);
}

void TargetTrackingNode::update_timer(int update_interval)
{
    timer_->cancel();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&line_filtering_node::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Updated timer with %d ms update interval", update_interval);
}
