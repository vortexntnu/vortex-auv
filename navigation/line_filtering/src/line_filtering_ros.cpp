#include <line_filtering/line_filtering_ros.hpp>

using std::placeholders::_1;

LineFilteringNode::LineFilteringNode() : Node("line_filtering_node") {
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
    declare_parameter<double>("std_dynmod", 0.2);
    declare_parameter<double>("std_sensor", 0.5);
    declare_parameter<double>("connected_lines_threshold", 0.5);
    declare_parameter<double>("crossing_min_angle", 0.5236);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    target_frame_ =
        this->declare_parameter<std::string>("target_frame", "odom");
    auto line_sub_topic = this->declare_parameter<std::string>(
        "line_sub_topic", "/line/pose_array");
    auto camera_info_sub_topic = this->declare_parameter<std::string>(
        "camera_info_sub_topic", "/cam_down/camera_info");
    auto dvl_depth_sub_topic = this->declare_parameter<std::string>(
        "dvl_depth_sub_topic", "/dvl/depth");
    auto pose_array_pub_topic = this->declare_parameter<std::string>(
        "pose_array_pub_topic", "/filtered_pose_array");
    auto odom_sub_topic =
        this->declare_parameter<std::string>("odom_sub_topic", "/orca/odom");
    debug_visualization_ = this->declare_parameter("debug_visualization", true);

    auto line_point_pub_topic = this->declare_parameter<std::string>(
        "line_point_pub_topic", "/line/point");
    auto line_intersection_pub_topic = this->declare_parameter<std::string>(
        "line_intersection_pub_topic", "/line/intersection");

    line_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        line_point_pub_topic, qos_sensor_data);

    line_intersection_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        line_intersection_pub_topic, qos_sensor_data);

    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());

    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10))
                          .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    line_sub_.subscribe(this, line_sub_topic, qos.get_rmw_qos_profile());

    tf2_filter_ =
        std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseArray>>(
            line_sub_, *tf2_buffer_, target_frame_, 100,
            this->get_node_logging_interface(),
            this->get_node_clock_interface());

    tf2_filter_->registerCallback(
        std::bind(&LineFilteringNode::line_callback, this, _1));

    // Subscriptions
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_sub_topic, qos_sensor_data,
        std::bind(&LineFilteringNode::camera_info_callback, this,
                  std::placeholders::_1));

    depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        dvl_depth_sub_topic, qos_sensor_data,
        std::bind(&LineFilteringNode::depth_callback, this,
                  std::placeholders::_1));

    pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        line_sub_topic, qos_sensor_data,
        std::bind(&LineFilteringNode::line_callback, this,
                  std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_sub_topic, qos_sensor_data,
        std::bind(&LineFilteringNode::odom_callback, this,
                  std::placeholders::_1));

    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pose_array_pub_topic, qos_sensor_data);

    line_termination_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/line/termination", 1);

    if (debug_visualization_) {
        point_1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/line/point_1", qos_sensor_data);
        point_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/line/point_2", qos_sensor_data);
        point_3_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/line/point_3", qos_sensor_data);
        point_4_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/line/point_4", qos_sensor_data);

        line_params_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/tracks/param", qos_sensor_data);
        line_points_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/tracks/points", qos_sensor_data);

        scene_update_line_pub_ =
            this->create_publisher<foxglove_msgs::msg::SceneUpdate>(
                "/scene_update", qos_sensor_data);

        scene_update_intersection_pub_ =
            this->create_publisher<foxglove_msgs::msg::SceneUpdate>(
                "/scene_update_intersection", qos_sensor_data);

        line_intersection_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/line/intersection_pose", qos_sensor_data);
        
        line_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/line/pose", qos_sensor_data);
        
        
    }

    depth_.data = -1.0;

    // Initialize track manager
    double std_dynmod = get_parameter("std_dynmod").as_double();
    double std_sensor = get_parameter("std_sensor").as_double();

    line_tracker_ = TrackManager();
    line_tracker_.set_dyn_model(std_dynmod);
    line_tracker_.set_sensor_model(std_sensor);

    line_intersection_tracker_ = TrackManager();
    line_intersection_tracker_.set_dyn_model(std_dynmod);
    line_intersection_tracker_.set_sensor_model(std_sensor);

    // set timer
    int update_interval = get_parameter("update_interval_ms").as_int();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_interval),
        std::bind(&LineFilteringNode::timer_callback, this));
}

void LineFilteringNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    RCLCPP_INFO(this->get_logger(), "Camera Intrinsic Matrix initialized.");
    camera_info_received_ = true;
    camera_info_sub_.reset();
}

void LineFilteringNode::line_callback(
    const std::shared_ptr<const geometry_msgs::msg::PoseArray>& msg) {
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
        geometry_msgs::msg::TransformStamped transform =
            tf2_buffer_->lookupTransform(target_frame_, msg->header.frame_id,
                                         tf2::TimePointZero);

        size_t i = 0;
        size_t size = msg->poses.size();
        measurements_ = Eigen::Array<double, 2, Eigen::Dynamic>(2, size);
        line_params_ = Eigen::Array<double, 2, Eigen::Dynamic>(2, size / 2);

        for (const auto& pose : msg->poses) {
            double u = pose.position.x;
            double v = pose.position.y;

            // Transform 2D (u, v) to 3D (X, Y, Z) using the camera intrinsic
            // matrix
            double X = (u - K_.at<double>(0, 2)) * depth / K_.at<double>(0, 0);
            double Y = (v - K_.at<double>(1, 2)) * depth / K_.at<double>(1, 1);

            tf2::Vector3 point(X, Y, depth);

            // Apply the transform to the position only
            tf2::Vector3 transformed =
                tf2::Transform(
                    tf2::Quaternion(transform.transform.rotation.x,
                                    transform.transform.rotation.y,
                                    transform.transform.rotation.z,
                                    transform.transform.rotation.w),
                    tf2::Vector3(transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 transform.transform.translation.z)) *
                point;

            measurements_.col(i) << transformed.x(), transformed.y();
            if (i % 2 == 1) {
                line_params_.col((i - 1) / 2) = (measurements_.col(i - 1) +
                                                measurements_.col(i)) / 2.0;
            }

            // Publish the transformed points for visualization
            if (debug_visualization_) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = msg->header.stamp;
                pose_msg.header.frame_id = target_frame_;
                pose_msg.pose.position.x = transformed.x();
                pose_msg.pose.position.y = transformed.y();
                pose_msg.pose.position.z = transformed.z();

                switch (i) {
                    case 0:
                        point_1_->publish(pose_msg);
                        break;
                    case 1:
                        point_2_->publish(pose_msg);
                        break;
                    case 2:
                        point_3_->publish(pose_msg);
                        break;
                    case 3:
                        point_4_->publish(pose_msg);
                        break;
                    default:
                        break;
                }
            }

            i++;
        }

    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

void LineFilteringNode::depth_callback(
    const std_msgs::msg::Float64::SharedPtr msg) {
    depth_.data = msg->data;
}

void LineFilteringNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    orca_pose_ = msg->pose.pose;
}

void LineFilteringNode::timer_callback() {
    // get parameters
    int update_interval = get_parameter("update_interval_ms").as_int();
    double confirmation_threshold =
        get_parameter("confirmation_threshold").as_double();
    double gate_threshold = get_parameter("gate_threshold").as_double();
    double min_gate_threshold = get_parameter("min_gate_threshold").as_double();
    double max_gate_threshold = get_parameter("max_gate_threshold").as_double();
    double prob_of_detection =
        get_parameter("probability_of_detection").as_double();
    double prob_of_survival =
        get_parameter("probability_of_survival").as_double();
    double clutter_intensity = get_parameter("clutter_rate").as_double();
    double deletion_threshold = get_parameter("deletion_threshold").as_double();
    double initial_existence_probability =
        get_parameter("initial_existence_probability").as_double();

    // Update line tracks
    line_tracker_.update_line_tracks(
        measurements_, line_params_, update_interval, confirmation_threshold,
        gate_threshold, min_gate_threshold, max_gate_threshold,
        prob_of_detection, prob_of_survival, clutter_intensity,
        initial_existence_probability);

    measurements_.resize(2, 0);
    line_params_.resize(2, 0);

    // delete tracks
    line_tracker_.delete_tracks(deletion_threshold);

    // find line crossings
    find_new_line_intersections();

    line_intersection_tracker_.update_line_intersection_tracks(
        current_line_intersections_, current_intersection_ids_, current_line_intersection_points_,
        update_interval, confirmation_threshold, gate_threshold,
        min_gate_threshold, max_gate_threshold, prob_of_detection,
        prob_of_survival, clutter_intensity, initial_existence_probability);

    current_line_intersections_.resize(2, 0);
    current_intersection_ids_.resize(2, 0);

    line_intersection_tracker_.delete_tracks(deletion_threshold);

    if (debug_visualization_) {
        auto scene_update_line = visualize_track_gates(line_tracker_.get_tracks(),
                                                  this->now(), target_frame_,
                                                    gate_threshold, min_gate_threshold,
                                                    max_gate_threshold, true,
                                                    orca_pose_.position.z, depth_.data);

        scene_update_line_pub_->publish(scene_update_line);

        auto scene_update_intersection =
            visualize_track_gates(line_intersection_tracker_.get_tracks(),
                                  this->now(), target_frame_, gate_threshold,
                                  min_gate_threshold, max_gate_threshold, false,
                                  orca_pose_.position.z, depth_.data);

        auto marker_array = visualize_line_tracks(line_tracker_.get_tracks(),
                                                  this->now(), target_frame_, orca_pose_.position.z, depth_.data);
        line_points_pub_->publish(marker_array);

        scene_update_intersection_pub_->publish(scene_update_intersection);
        
    }
    if (new_intersection_available()) {
        publish_intersection();
        return;
    }
    if(line_tracker_.get_tracks().size() == 0){
        return;
    }
    if (used_line_intersections_.size() == 0) {
        find_and_publish_initial_waypoint();
        return;
    }
    if (used_line_intersections_.size() > 0) {
       publish_waypoint();
    }

    
}

void LineFilteringNode::publish_intersection(){

    for (const auto& int_track : line_intersection_tracker_.get_tracks()) {
        if (!int_track.confirmed) {
            continue;
        }
        set_next_line(int_track);
        
        used_line_intersections_.push_back(LineIntersection{int_track.state.mean()(0), int_track.state.mean()(1), int_track.id1, int_track.id2, int_track.line_points});
        geometry_msgs::msg::PointStamped intersection_point;
        intersection_point.header.frame_id = target_frame_;
        intersection_point.header.stamp = this->now();
        intersection_point.point.x = int_track.state.mean()(0);
        intersection_point.point.y = int_track.state.mean()(1);
        // Remove so not used again
        line_intersection_tracker_.delete_track_by_id(int_track.id);
        line_intersection_pub_->publish(intersection_point);
        if (debug_visualization_) {
            geometry_msgs::msg::PoseStamped intersection_pose;
            intersection_pose.header.frame_id = target_frame_;
            intersection_pose.header.stamp = this->now();
            intersection_pose.pose.position.x = intersection_point.point.x;
            intersection_pose.pose.position.y = intersection_point.point.y;
            intersection_pose.pose.position.z = orca_pose_.position.z;
            line_intersection_pose_pub_->publish(intersection_pose);
        }
        return;    
    }
}

bool LineFilteringNode::new_intersection_available() {
    for (const auto& track : line_intersection_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }
        return true;
    }
    return false;
}

void LineFilteringNode::set_next_line(const Track& int_track) {
    // Get the yaw from the current orientation.
    tf2::Quaternion q(orca_pose_.orientation.x, orca_pose_.orientation.y,
                        orca_pose_.orientation.z, orca_pose_.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Get the endpoints of the line and the intersection point.
    Eigen::Vector2d line1 = int_track.line_points.col(0);
    Eigen::Vector2d line2 = int_track.line_points.col(1);
    Eigen::Vector2d intersection = int_track.state.mean();

    // Compute vectors from the line endpoints to the intersection.
    Eigen::Vector2d vec1 = (intersection - line1).normalized(); // corresponds to id1
    Eigen::Vector2d vec2 = (intersection - line2).normalized(); // corresponds to id2

    // Create the yaw direction vector.
    Eigen::Vector2d yaw_dir(std::cos(yaw), std::sin(yaw));

    // Compute the angles between the yaw direction and each vector.
    double dot1 = std::max(-1.0, std::min(1.0, yaw_dir.dot(vec1)));
    double dot2 = std::max(-1.0, std::min(1.0, yaw_dir.dot(vec2)));
    double angle1 = std::acos(dot1);
    double angle2 = std::acos(dot2);

    // First check: if we've already processed enough lines and one of the ids matches,
    // choose the "other" line. Also set next_line_yaw_ based on the corresponding vector.
    if (current_line_id_counter_ > 5 && (current_line_id_ == int_track.id1 || current_line_id_ == int_track.id2)) {
        if (current_line_id_ == int_track.id1) {
            next_line_id_ = int_track.id2;
            // Use vec2 (from line2 to intersection) for id2.
            next_line_yaw_ = std::atan2(vec2.y(), vec2.x());
            return;
        } else if (current_line_id_ == int_track.id2) {
            next_line_id_ = int_track.id1;
            // Use vec1 (from line1 to intersection) for id1.
            next_line_yaw_ = std::atan2(vec1.y(), vec1.x());
            return;
        }
    }

    // Otherwise, choose the candidate with the biggest angle difference from the current yaw.
    if (angle1 > angle2) {
        next_line_id_ = int_track.id1;
        next_line_yaw_ = std::atan2(vec1.y(), vec1.x());
    } else {
        next_line_id_ = int_track.id2;
        next_line_yaw_ = std::atan2(vec2.y(), vec2.x());
    }
}


void LineFilteringNode::find_new_line_intersections() {
    // Define threshold for connected lines
    double connected_threshold = get_parameter("connected_lines_threshold").as_double();
    double min_angle = get_parameter("crossing_min_angle").as_double();
    std::vector<Track> tracks = line_tracker_.get_tracks();
    std::vector<LineIntersection> intersections;

    for (const auto& track : tracks) {
        if (!track.confirmed) continue;

        for (const auto& track2 : tracks) {
            if (!track2.confirmed) continue;
            if (track.id == track2.id) continue;

            Eigen::Matrix<double, 2, 2> line1_points = track.line_points;
            Eigen::Matrix<double, 2, 2> line2_points = track2.line_points;
            Eigen::Vector2d intersection;

            if (!find_intersection(line1_points, line2_points, intersection, min_angle)) {
                continue;
            }

            Eigen::Vector2d intersection_point_line1;
            Eigen::Vector2d intersection_point_line2;
            LineIntersection line_intersection;

            // Store points not used in intersection in line_points
            if ((intersection - line1_points.col(0)).norm() < (intersection - line1_points.col(1)).norm()) {
                intersection_point_line1 = line1_points.col(0);
                line_intersection.line_points.col(0) << line1_points.col(1);
            } else {
                intersection_point_line1 = line1_points.col(1);
                line_intersection.line_points.col(0) << line1_points.col(0);
            }

            if ((intersection - line2_points.col(0)).norm() < (intersection - line2_points.col(1)).norm()) {
                intersection_point_line2 = line2_points.col(0);
                line_intersection.line_points.col(1) << line2_points.col(1);
            } else {
                intersection_point_line2 = line2_points.col(1);
                line_intersection.line_points.col(1) << line2_points.col(0);
            }

            if ((intersection_point_line1 - intersection_point_line2).norm() < connected_threshold) {
                line_intersection.x = intersection(0);
                line_intersection.y = intersection(1);
                line_intersection.id1 = track.id;
                line_intersection.id2 = track2.id;

                
                // Check if the intersection already exists by comparing both ids to the existing intersections
                if (std::find(used_line_intersections_.begin(), used_line_intersections_.end(), line_intersection) != used_line_intersections_.end()) {
                    continue;
                }
                intersections.push_back(line_intersection);
                
            }            

        }
    }
    int size = intersections.size();
    current_line_intersections_ = Eigen::Array<double, 2, Eigen::Dynamic>(2, size);
    current_intersection_ids_ = Eigen::Array<int, 2, Eigen::Dynamic>(2, size);
    current_line_intersection_points_ = Eigen::Array<double, 2, Eigen::Dynamic>(2, 2*size);

    for (size_t i = 0; i < intersections.size(); i++) {
        current_line_intersections_.col(i) << intersections.at(i).x, intersections.at(i).y;
        current_intersection_ids_.col(i) << intersections.at(i).id1, intersections.at(i).id2;
        current_line_intersection_points_.col(2*i) << intersections.at(i).line_points.col(0);
        current_line_intersection_points_.col(2*i+1) << intersections.at(i).line_points.col(1);
    }

}

bool LineFilteringNode::find_intersection(const Eigen::Matrix<double, 2, 2>& line1, const Eigen::Matrix<double, 2, 2>& line2, Eigen::Vector2d& intersection, double min_angle) {
    // Extract points from line1 and line2
    double x1 = line1(0, 0), y1 = line1(1, 0);  // Line 1: Point 1 (x1, y1)
    double x2 = line1(0, 1), y2 = line1(1, 1);  // Line 1: Point 2 (x2, y2)
    
    double x3 = line2(0, 0), y3 = line2(1, 0);  // Line 2: Point 1 (x3, y3)
    double x4 = line2(0, 1), y4 = line2(1, 1);  // Line 2: Point 2 (x4, y4)
    
    // Compute the direction vectors of both lines
    Eigen::Vector2d dir1(x2 - x1, y2 - y1);  // Direction of Line 1
    Eigen::Vector2d dir2(x4 - x3, y4 - y3);  // Direction of Line 2
    
    // Compute the dot product and magnitudes
    double dot_product = dir1.dot(dir2);
    double mag1 = dir1.norm();
    double mag2 = dir2.norm();
    
    // Calculate the cosine of the angle between the two vectors
    double cos_theta = dot_product / (mag1 * mag2);
    
    // Ensure the cosine value is in the valid range [-1, 1] to avoid numerical errors
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    
    // Calculate the angle in radians
    double angle_rad = std::acos(cos_theta);
    
    // Convert angle to degrees
    // double angle_deg = angle_rad * (180.0 / M_PI);  // Convert radians to degrees manually

    // Check if the angle is smaller than the minimum angle
    if (angle_rad < min_angle) {
        return false;
    }

    // Compute the slopes (m1 and m2) of the lines
    double m1 = (y2 - y1) / (x2 - x1);
    double m2 = (y4 - y3) / (x4 - x3);

    // If the lines are parallel (m1 == m2), there's no intersection
    if (m1 == m2) {
        return false;
    }

    // Use the equation to solve for x and y (intersection point)
    double x = (y3 - y1 + m1 * x1 - m2 * x3) / (m1 - m2);
    double y = m1 * (x - x1) + y1;

    // Store the intersection point in the 'intersection' vector
    intersection << x, y;
    
    return true;
}

int LineFilteringNode::get_track_by_id(Track& line_track, int id) {
    for (const auto& track : line_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }
        if (track.id == id) {
            line_track = track;
            return id;
        }
    }
    return -1;
}

void LineFilteringNode::find_and_publish_initial_waypoint(){
    // Get the robot's position and orientation
    geometry_msgs::msg::Pose Pose_orca = orca_pose_;
    double orca_x = Pose_orca.position.x;
    double orca_y = Pose_orca.position.y;

    // Convert quaternion to yaw (robot's heading)
    tf2::Quaternion q(Pose_orca.orientation.x, Pose_orca.orientation.y,
                      Pose_orca.orientation.z, Pose_orca.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    auto tracks = line_tracker_.get_tracks();

    Track chosen_track;

    if (tracks.size() == 0) {
        return;
    }

    if (tracks.size() == 1){
        chosen_track = tracks.front();
    }

    if (tracks.size() > 1) {
        double min_distance = std::numeric_limits<double>::max();
        chosen_track = tracks.front();

        // Robot's current position.
        double rx = orca_x;
        double ry = orca_y;

        for (const auto &track : tracks) {
            // Get endpoints for this track.
            Eigen::Vector2d a = track.line_points.col(0);
            Eigen::Vector2d b = track.line_points.col(1);

            double ax = a(0);
            double ay = a(1);
            double bx = b(0);
            double by = b(1);
    
            double ABx = bx - ax;
            double ABy = by - ay;
            double normAB = std::hypot(ABx, ABy);
            if (normAB < 1e-6) {
                // If the endpoints are nearly identical, skip this track.
                continue;
            }
    
            // Compute the perpendicular distance from the drone to the infinite line.
            // This is |(B-A) x (R-A)| / |B-A|
            double ARx = rx - ax;
            double ARy = ry - ay;
            double cross_mag = std::fabs(ABx * ARy - ABy * ARx);
            double distance = cross_mag / normAB;
    
            if (distance < min_distance) {
                min_distance = distance;
                chosen_track = track;
            }
        }
    }

   // Robot's current position
   double rx = orca_x;
   double ry = orca_y;

   // Determine if the robot is between the endpoints
   double ax = chosen_track.line_points(0, 0);
   double ay = chosen_track.line_points(1, 0);
   double bx = chosen_track.line_points(0, 1);
   double by = chosen_track.line_points(1, 1);

   double ABx = bx - ax;
   double ABy = by - ay;
   double ARx = rx - ax;
   double ARy = ry - ay;

   // Calculate the projection factor (t) of the robot's position onto the line.
   double ab_squared = ABx * ABx + ABy * ABy;
   // Avoid division by zero when both endpoints are the same.
   double t = (ab_squared == 0.0 ? 0.0 : ((ARx * ABx + ARy * ABy) / ab_squared));

   // The robot is "between" if its projection lies between 0 and 1.
   bool robotBetween = (t >= 0.0 && t <= 1.0);

   geometry_msgs::msg::PointStamped chosen_point;

   if (robotBetween) {
       // If the robot is between the endpoints, choose the one that aligns best with our heading.

       // Compute vectors from the robot to each endpoint.
       double dx1 = ax - rx;
       double dy1 = ay - ry;
       double dx2 = bx - rx;
       double dy2 = by - ry;

       // Calculate the angle from the robot to each point.
       double angle1 = std::atan2(dy1, dx1);
       double angle2 = std::atan2(dy2, dx2);

       // Determine the angular differences, properly wrapped between -pi and pi.
       double diff1 = std::fabs(std::atan2(std::sin(angle1 - yaw), std::cos(angle1 - yaw)));
       double diff2 = std::fabs(std::atan2(std::sin(angle2 - yaw), std::cos(angle2 - yaw)));

       if (diff1 < diff2) {
           chosen_point.point.x = ax;
           chosen_point.point.y = ay;
       } else {
           chosen_point.point.x = bx;
           chosen_point.point.y = by;
       }

   } else {
       // Otherwise, choose the point that is furthest away.
       double dx1 = ax - rx;
       double dy1 = ay - ry;
       double dx2 = bx - rx;
       double dy2 = by - ry;

       double dist1 = std::hypot(dx1, dy1);
       double dist2 = std::hypot(dx2, dy2);

         if (dist1 > dist2) {
              chosen_point.point.x = ax;
              chosen_point.point.y = ay;
         } else {
              chosen_point.point.x = bx;
              chosen_point.point.y = by;
         }
   }

   if (current_line_id_ == chosen_track.id) {
       current_line_id_counter_++;
    } else {
         current_line_id_ = chosen_track.id;
         current_line_id_counter_ = 0;
    }

   chosen_point.header.stamp = this->now();
   chosen_point.header.frame_id = target_frame_;
   chosen_point.point.z = orca_pose_.position.z + depth_.data;
   line_point_pub_->publish(chosen_point);

   if (debug_visualization_) {
       geometry_msgs::msg::PoseStamped pose_msg;
       pose_msg.header.stamp = this->now();
       pose_msg.header.frame_id = target_frame_;
       pose_msg.pose.position.x = chosen_point.point.x;
       pose_msg.pose.position.y = chosen_point.point.y;
       pose_msg.pose.position.z = chosen_point.point.z;
       line_pose_pub_->publish(pose_msg);
   }
}

void LineFilteringNode::publish_waypoint(){
    auto intersection = used_line_intersections_.back();

    Track next_line;
    if(get_track_by_id(next_line, next_line_id_) == -1){
        get_track_by_yaw(next_line);
    }
    if(current_line_id_ == next_line.id1){
        current_line_id_counter_++;
    } else {
        current_line_id_ = next_line.id1;
        current_line_id_counter_ = 0;
    }
    geometry_msgs::msg::PointStamped line_point;
    line_point.header.frame_id = target_frame_;
    line_point.header.stamp = this->now();
    Eigen::Vector2d next_line_p1 = next_line.line_points.col(0);
    Eigen::Vector2d next_line_p2 = next_line.line_points.col(1);
    double distance1 = std::hypot(intersection.x - next_line_p1(0), intersection.y - next_line_p1(1));
    double distance2 = std::hypot(intersection.x - next_line_p2(0), intersection.y - next_line_p2(1));
    if (distance1 > distance2) {
        line_point.point.x = next_line_p1(0);
        line_point.point.y = next_line_p1(1);
    } else {
        line_point.point.x = next_line_p2(0);
        line_point.point.y = next_line_p2(1);
    }
    line_point.point.z = orca_pose_.position.z;
    line_point_pub_->publish(line_point);
    if(debug_visualization_){
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = target_frame_;
        pose_msg.pose.position.x = line_point.point.x;
        pose_msg.pose.position.y = line_point.point.y;
        pose_msg.pose.position.z = line_point.point.z;
        line_pose_pub_->publish(pose_msg);
    }        
}

void LineFilteringNode::get_track_by_yaw(Track& line_track) {
    double prev_yaw = next_line_yaw_;
    double min_diff = std::numeric_limits<double>::max();
    for (const auto& track : line_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }
        Eigen::Vector2d a = track.line_points.col(0);
        Eigen::Vector2d b = track.line_points.col(1);
        double angle = std::atan2(b(1) - a(1), b(0) - a(0));
        double diff = std::fabs(std::atan2(std::sin(angle - prev_yaw), std::cos(angle - prev_yaw)));
        if (diff < min_diff) {
            min_diff = diff;
            line_track = track;
        }
    }
}

void LineFilteringNode::termination_check() {
    Track line_track;
    if (get_track_by_id(line_track, current_line_id_) == -1) {
        return;
        termination_counter_ = 0;
    }
    Eigen::Vector2d line_p1 = line_track.line_points.col(0);
    Eigen::Vector2d line_p2 = line_track.line_points.col(1);
    auto intersection = used_line_intersections_.back();
    Eigen::Vector2d orca_position(orca_pose_.position.x, orca_pose_.position.y);
    double distance1 = std::hypot(intersection.x - line_p1(0), intersection.y - line_p1(1));
    double distance2 = std::hypot(intersection.x - line_p2(0), intersection.y - line_p2(1));
    Eigen::Vector2d endpoint;
    if (distance1 > distance2) {
        endpoint = line_p1;
    } else {
        endpoint = line_p2;
    }
    double distance = std::hypot(orca_position(0) - endpoint(0), orca_position(1) - endpoint(1));
    if (distance < 0.5) {
        termination_counter_++;
    } else {
        termination_counter_ = 0;
    }
    if (termination_counter_ > 30) {
        line_termination_pub_->publish(std_msgs::msg::Bool());
    }
}

void LineFilteringNode::update_timer(int update_interval) {
    timer_->cancel();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_interval),
        std::bind(&LineFilteringNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Updated timer with %d ms update interval",
                update_interval);
}

void LineFilteringNode::update_dyn_model(double std_dynmod) {
    line_tracker_.set_dyn_model(std_dynmod);
    RCLCPP_INFO(this->get_logger(), "Updated dynamic model");
}

void LineFilteringNode::update_sensor_model(double std_measurement) {
    line_tracker_.set_sensor_model(std_measurement);
    RCLCPP_INFO(this->get_logger(), "Updated sensor model");
}
