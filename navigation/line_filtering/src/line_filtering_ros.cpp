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
                line_params_.col((i - 1) / 2) =
                    get_line_params(measurements_.block<2, 2>(0, i - 1));
            }

            // Publish the transformed points for visualization
            if (debug_visualization_) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header = msg->header;
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
        visualize_tracks();
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

void LineFilteringNode::visualize_tracks() {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = this->now();
    marker.ns = "track_points";
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    for (const auto& track : line_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }

        geometry_msgs::msg::Point start;
        start.x = track.line_points(0, 0);
        start.y = track.line_points(1, 0);
        start.z = orca_pose_.position.z + depth_.data;

        geometry_msgs::msg::Point end;
        end.x = track.line_points(0, 1);
        end.y = track.line_points(1, 1);
        end.z = orca_pose_.position.z + depth_.data;

        marker.points.push_back(start);
        marker.points.push_back(end);
    }

    marker_array.markers.push_back(marker);
    line_points_pub_->publish(marker_array);

    visualization_msgs::msg::MarkerArray line_params_array;
    visualization_msgs::msg::Marker line_params;
    line_params.header.frame_id = target_frame_;
    line_params.header.stamp = this->now();
    line_params.ns = "line_params";
    line_params.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_params.action = visualization_msgs::msg::Marker::ADD;
    line_params.pose.orientation.w = 1.0;
    line_params.scale.x = 0.05;
    line_params.color.g = 1.0;
    line_params.color.a = 1.0;

    // Length of the segment to visualize (adjust as needed)
    const float segment_length = 20.0f;

    for (const auto& track : line_tracker_.get_tracks()) {
        if (!track.confirmed) {
            continue;
        }

        // Extract line parameters
        float line_angle = track.state.mean()(0);
        float line_distance = track.state.mean()(1);

        // Compute the normal and tangent vectors of the line
        float cos_angle = std::cos(line_angle);
        float sin_angle = std::sin(line_angle);
        // Unit normal (points from the origin toward the line)
        Eigen::Vector2f n(cos_angle, sin_angle);
        // Tangent (direction along the line)
        Eigen::Vector2f t(-sin_angle, cos_angle);

        // Get the current position from orca_pose_
        float pose_x = orca_pose_.position.x;
        float pose_y = orca_pose_.position.y;
        Eigen::Vector2f p(pose_x, pose_y);

        // Compute the projection of p onto the line
        float n_dot_p = n.dot(p);
        float distance_from_line = n_dot_p - line_distance;
        Eigen::Vector2f p_closest = p - distance_from_line * n;

        // Define endpoints along the tangent direction (half segment in each
        // direction)
        Eigen::Vector2f start_vec = p_closest + (segment_length / 2.0f) * t;
        Eigen::Vector2f end_vec = p_closest - (segment_length / 2.0f) * t;

        geometry_msgs::msg::Point start;
        start.x = start_vec.x();
        start.y = start_vec.y();
        start.z = orca_pose_.position.z +
                  depth_.data;  // same z as your other markers

        geometry_msgs::msg::Point end;
        end.x = end_vec.x();
        end.y = end_vec.y();
        end.z = orca_pose_.position.z + depth_.data;

        line_params.points.push_back(start);
        line_params.points.push_back(end);
    }
    line_params_array.markers.push_back(line_params);
    line_params_pub_->publish(line_params_array);
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

            // Extract line parameters (angle, distance)
            Eigen::Vector2d line1_params = track.state.mean();
            Eigen::Vector2d line2_params = track2.state.mean();

            double theta1 = line1_params(0);
            double d1 = line1_params(1);
            double theta2 = line2_params(0);
            double d2 = line2_params(1);

            // Check if the lines are not nearly parallel
            if (std::abs(theta1 - theta2) < min_angle) {
                continue;
            }

            // Form the system Ax = B
            Eigen::Matrix2d A;
            A << std::cos(theta1), std::sin(theta1),
                    std::cos(theta2), std::sin(theta2);

            Eigen::Vector2d B(d1, d2);

            Eigen::Vector2d intersection;
            // Solve for intersection point (x, y)
            if (A.determinant() != 0) {  // Ensure the lines are not parallel
                intersection = A.inverse() * B;
            } else {
                continue;
            }

            Eigen::Matrix<double,2,2> line1_points = track.line_points;
            Eigen::Matrix<double,2,2> line2_points = track2.line_points;
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
