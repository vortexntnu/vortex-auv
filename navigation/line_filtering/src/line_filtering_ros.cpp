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

Eigen::Vector2d LineFilteringNode::get_line_params(
    Eigen::Matrix<double, 2, 2> line_points) {
    // Extract the two points (x1, y1) and (x2, y2)
    Eigen::Vector2d point1 = line_points.col(0);
    Eigen::Vector2d point2 = line_points.col(1);

    // Direction vector of the line
    Eigen::Vector2d direction = point2 - point1;

    // Normal vector to the line (perpendicular to the direction vector)
    Eigen::Vector2d normal(-direction.y(), direction.x());
    normal.normalize();  // Ensure the normal vector is of unit length

    // Compute the distance from the origin to the line
    // Use point1 for this calculation: distance = dot(normal, point1)
    double distance_to_origin = std::abs(normal.dot(point1));

    // Compute the angle of the normal vector with respect to the x-axis
    double angle = std::atan2(normal.y(), normal.x());

    // Return the parameters: [angle, distance]
    return Eigen::Vector2d(angle, distance_to_origin);
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

    // for (const auto& track : line_tracker_.get_tracks()) {
    //     if (!track.confirmed) {
    //         continue;
    //     }
    //     RCLCPP_INFO(this->get_logger(), "Line found with id: %d", track.id);
    //     RCLCPP_INFO(this->get_logger(), "existance probability: %f",
    //                 track.existence_probability);
    //     RCLCPP_INFO(this->get_logger(), "line parameters: angle: %f, distance: %f",
    //                 track.state.mean()(0), track.state.mean()(1));
    
    // }

    // find line crossings
    find_line_intersections();

    line_intersection_tracker_.update_line_intersection_tracks(
        current_line_intersections_, current_intersection_ids_,
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

    int line_intersection_id = find_intersection_id();

    if (line_intersection_id != -1) {
        RCLCPP_INFO(this->get_logger(), "Line intersection found with id: %d",
                    line_intersection_id);
        auto track = line_intersection_tracker_.get_track(line_intersection_id);
        geometry_msgs::msg::PointStamped intersection_point;
        intersection_point.header.frame_id = target_frame_;
        intersection_point.header.stamp = this->now();
        intersection_point.point.x = track.state.mean()(0);
        intersection_point.point.y = track.state.mean()(1);
        line_intersection_pub_->publish(intersection_point);
        
    }

    // select_line();
}

void LineFilteringNode::find_line_intersections() {
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

            if ((intersection - line1_points.col(0)).norm() < (intersection - line1_points.col(1)).norm()) {
                intersection_point_line1 = line1_points.col(0);
            } else {
                intersection_point_line1 = line1_points.col(1);
            }

            if ((intersection - line2_points.col(0)).norm() < (intersection - line2_points.col(1)).norm()) {
                intersection_point_line2 = line2_points.col(0);
            } else {
                intersection_point_line2 = line2_points.col(1);
            }

            if ((intersection_point_line1 - intersection_point_line2).norm() < connected_threshold) {
                LineIntersection line_intersection;
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

    for (size_t i = 0; i < intersections.size(); i++) {
        current_line_intersections_.col(i) << intersections.at(i).x, intersections.at(i).y;
        current_intersection_ids_.col(i) << intersections.at(i).id1, intersections.at(i).id2;
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

int LineFilteringNode::find_intersection_id() {
    for (const auto& track : line_intersection_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }
        used_line_intersections_.push_back(LineIntersection{track.state.mean()(0), track.state.mean()(1), track.id1, track.id2});
        return track.id;
    }
    return -1;
}

void LineFilteringNode::select_line() {
    // Get the robot's position and orientation
    geometry_msgs::msg::Pose Pose_orca = orca_pose_;
    double position_x = Pose_orca.position.x;
    double position_y = Pose_orca.position.y;

    // Convert quaternion to yaw (robot's heading)
    tf2::Quaternion q(Pose_orca.orientation.x, Pose_orca.orientation.y,
                      Pose_orca.orientation.z, Pose_orca.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double threshold_distance = 0.1;  // Distance threshold
    double angular_threshold = 0.1;   // Angular threshold in radians

    const int treshold_count = 5;

    for (const auto& track : line_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }

        geometry_msgs::msg::Pose line_start;
        line_start.position.x = track.line_points(0, 0);
        line_start.position.y = track.line_points(1, 0);

        geometry_msgs::msg::Pose line_end;
        line_end.position.x = track.line_points(0, 1);
        line_end.position.y = track.line_points(1, 1);

        // Compute the line's direction vector
        double line_dir_x = line_end.position.x - line_start.position.x;
        double line_dir_y = line_end.position.y - line_start.position.y;

        // Normalize the direction vector
        double line_length =
            std::sqrt(line_dir_x * line_dir_x + line_dir_y * line_dir_y);
        line_dir_x /= line_length;
        line_dir_y /= line_length;

        // Compute the vector from line start to the robot's position
        double robot_vec_x = position_x - line_start.position.x;
        double robot_vec_y = position_y - line_start.position.y;

        // Project the robot's vector onto the line's direction vector (dot
        // product) double projection = (robot_vec_x * line_dir_x) +
        // (robot_vec_y * line_dir_y);

        // // Check if the robot is on the line segment
        // if (projection < 0 || projection > line_length) {
        //    continue;
        // }

        // Calculate the perpendicular distance from the robot to the line
        double perp_dist =
            std::abs(robot_vec_x * line_dir_y - robot_vec_y * line_dir_x);

        if (perp_dist > threshold_distance) {
            continue;
        }

        // Check if the robot's heading aligns with the line's direction
        double line_angle = std::atan2(line_dir_y, line_dir_x);
        double angular_difference =
            std::fmod(yaw - line_angle + M_PI, 2 * M_PI) - M_PI;

        if (std::abs(angular_difference) > angular_threshold) {
            continue;
        }

        if (track.id == current_id_) {
            current_track_points_.poses.clear();
            id_counter_++;
            current_track_points_.poses.push_back(line_start);
            current_track_points_.poses.push_back(line_end);
        } else {
            current_track_points_.poses.clear();
            current_id_ = track.id;
            id_counter_ = 0;
        }
    }
    if (id_counter_ >= treshold_count) {
        current_track_points_.header.stamp = this->now();
        current_track_points_.header.frame_id = target_frame_;
        current_track_points_.poses.front().position.z =
            orca_pose_.position.z + depth_.data;
        current_track_points_.poses.back().position.z =
            orca_pose_.position.z + depth_.data;
        pose_array_pub_->publish(current_track_points_);
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
