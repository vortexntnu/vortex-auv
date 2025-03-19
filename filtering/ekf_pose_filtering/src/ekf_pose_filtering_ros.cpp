#include <ekf_pose_filtering/ekf_pose_filtering_ros.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

EKFPoseFilteringNode::EKFPoseFilteringNode() : Node("ekf_pose_filtering_node") {
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "reset_ekf", std::bind(&EKFPoseFilteringNode::reset_EFK_state, this, _2));

    target_frame_ =
        this->declare_parameter<std::string>("target_frame");
    auto pose_sub_topic = this->declare_parameter<std::string>(
        "pose_sub_topic");

    enu_orientation_ = this->declare_parameter<bool>("enu_orientation");

    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());

    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10))
                          .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    pose_sub_.subscribe(this, pose_sub_topic, qos.get_rmw_qos_profile());

    tf2_filter_ = std::make_shared<
        tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>(
        pose_sub_, *tf2_buffer_, target_frame_, 100,
        this->get_node_logging_interface(), this->get_node_clock_interface());

    tf2_filter_->registerCallback(
        std::bind(&EKFPoseFilteringNode::pose_callback, this, _1));

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    auto transformed_pose_pub_topic = this->declare_parameter<std::string>(
        "transformed_pose_pub_topic");
    auto filtered_pose_pub_topic = this->declare_parameter<std::string>(
        "filtered_pose_pub_topic");

    transformed_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            transformed_pose_pub_topic, qos_sensor_data);
    filtered_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            filtered_pose_pub_topic, qos_sensor_data);

    double dynmod_stddev = this->declare_parameter<double>("dynmod_stddev");
    double sensmod_stddev = this->declare_parameter<double>("sensmod_stddev");
    dynamic_model_ = std::make_shared<DynMod>(dynmod_stddev);
    sensor_model_ = std::make_shared<SensMod>(sensmod_stddev);
}

void EKFPoseFilteringNode::pose_callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    try {
        tf2_buffer_->transform(*pose_msg, transformed_pose, target_frame_,
                               tf2::Duration(std::chrono::milliseconds(50)));
        transformed_pose.header.frame_id = target_frame_;
        if (enu_orientation_) {
            transformed_pose.pose.orientation =
                enu_to_ned_quaternion(transformed_pose.pose.orientation);
        }
        transformed_pose_pub_->publish(transformed_pose);
        filter_pose(transformed_pose);
        filtered_pose_pub_->publish(transformed_pose);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

void EKFPoseFilteringNode::filter_pose(
    geometry_msgs::msg::PoseStamped& pose_msg) {
    if (first_run_) {
        Eigen::Vector3d initial_measurement = {pose_msg.pose.position.x,
                                               pose_msg.pose.position.y,
                                               pose_msg.pose.position.z};
        previous_pose_est_ =
            Gauss3d(initial_measurement, Gauss3d::Mat_nn::Identity());
        previous_time_ = pose_msg.header.stamp;
        first_run_ = false;
        return;
    } else {
        rclcpp::Time current_time(pose_msg.header.stamp);
        rclcpp::Duration time_step = current_time - previous_time_;
        Eigen::Vector3d object_pose;
        object_pose(0) = pose_msg.pose.position.x;
        object_pose(1) = pose_msg.pose.position.y;
        object_pose(2) = pose_msg.pose.position.z;
        std::tie(object_pose_est_, std::ignore, std::ignore) =
            EKF::step(*dynamic_model_, *sensor_model_, time_step.seconds(),
                      previous_pose_est_, object_pose);
        previous_pose_est_ = object_pose_est_;
        previous_time_ = current_time;

        pose_msg.pose.position.x = object_pose_est_.mean()(0);
        pose_msg.pose.position.y = object_pose_est_.mean()(1);
        pose_msg.pose.position.z = object_pose_est_.mean()(2);
    }
}

geometry_msgs::msg::Quaternion EKFPoseFilteringNode::enu_to_ned_quaternion(
    const geometry_msgs::msg::Quaternion& enu_quat) {
    tf2::Quaternion tf_enu_quat(enu_quat.x, enu_quat.y, enu_quat.z, enu_quat.w);

    tf2::Quaternion q_rot_z, q_rot_x;
    q_rot_z.setRPY(0, 0, -M_PI / 2);
    q_rot_x.setRPY(M_PI, 0, 0);

    tf2::Quaternion tf_ned_quat = q_rot_x * q_rot_z * tf_enu_quat;

    geometry_msgs::msg::Quaternion ned_quat;
    ned_quat.x = tf_ned_quat.x();
    ned_quat.y = tf_ned_quat.y();
    ned_quat.z = tf_ned_quat.z();
    ned_quat.w = tf_ned_quat.w();

    return ned_quat;
}

void EKFPoseFilteringNode::reset_EFK_state(
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    first_run_ = false;
    previous_pose_est_ =
        Gauss3d(Gauss3d::Vec_n::Zero(), Gauss3d::Mat_nn::Identity());
    response->success = true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFPoseFilteringNode>());
    rclcpp::shutdown();
    return 0;
}
