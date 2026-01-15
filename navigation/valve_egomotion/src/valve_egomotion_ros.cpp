#include "valve_egomotion/valve_egomotion_ros.hpp"

#include <algorithm>
#include <chrono>

namespace valve_egomotion {

ValveEgomotionNode::ValveEgomotionNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("valve_egomotion", options), smoother_([&]() {
          SlidingWindowSO3Mean::Config cfg;
          cfg.win_size = this->declare_parameter<int>("window_size", 20);
          cfg.win_age_sec = this->declare_parameter<double>("max_age_sec", 1.0);
          cfg.huber_threshold_deg =
              this->declare_parameter<double>("huber_threshold_deg", 2.5);
          cfg.gate_threshold_deg =
              this->declare_parameter<double>("gate_threshold_deg", 15.0);
          cfg.use_mad_gate = this->declare_parameter<bool>("use_mad", false);
          return cfg;
      }()) {
    valve_topic_ = this->declare_parameter<std::string>(
        "valve_topic", "/aruco_detector/markers");
    ref_frame_ = this->declare_parameter<std::string>("ref_frame", "vo_ref");
    base_frame_ =
        this->declare_parameter<std::string>("base_frame", "base_link");
    cam_frame_ =
        this->declare_parameter<std::string>("cam_frame", "Orca/camera_front");

    publish_tf_ = this->declare_parameter<bool>("publish_tf", false);
    pub_rate_hz_ = this->declare_parameter<double>("pub_rate_hz", 30.0);
    lost_timeout_sec_ =
        this->declare_parameter<double>("lost_timeout_sec", 0.5);

    pub_pose_cov_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/landmark/pose_cov", rclcpp::QoS(10));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_marker_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        valve_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ValveEgomotionNode::onMarkers, this, std::placeholders::_1));

    last_det_stamp_ = this->now();
    last_pub_stamp_ = this->now();

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(1.0, pub_rate_hz_));
    pub_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ValveEgomotionNode::onPublishTimer, this));

    RCLCPP_INFO(
        this->get_logger(),
        "ValveEgomotionNode online. Sub: %s, base: %s, cam: %s, ref: %s",
        valve_topic_.c_str(), base_frame_.c_str(), cam_frame_.c_str(),
        ref_frame_.c_str());
}

void ValveEgomotionNode::onMarkers(
    const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (!msg || msg->poses.empty())
        return;

    // Take the first detection (keep your selector policy here if needed).
    tf2::Transform T_cam_marker;
    tf2::fromMsg(msg->poses[0], T_cam_marker);

    tf2::Transform T_base_cam;
    try {
        const auto ts = tf_buffer_->lookupTransform(base_frame_, cam_frame_,
                                                    tf2::TimePointZero);
        tf2::fromMsg(ts.transform, T_base_cam);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for TF %s <- %s: %s", base_frame_.c_str(),
                             cam_frame_.c_str(), ex.what());
        return;
    }

    const tf2::Transform T_base_marker_now = T_base_cam * T_cam_marker;

    if (!have_ref_) {
        // Anchor at first valid detection.
        T_base_marker0_ = T_base_marker_now;
        have_ref_ = true;
    }

    const tf2::Transform T_ref_base_meas =
        T_base_marker0_ * T_base_marker_now.inverse();

    smoother_.addSample(msg->header.stamp, T_ref_base_meas, 1.0);
    smoother_.prune(this->now());

    last_det_stamp_ = msg->header.stamp;
}

void ValveEgomotionNode::onPublishTimer() {
    const rclcpp::Time now = this->now();

    // If we haven't received detections for a while, stop publishing.
    if (!have_ref_)
        return;
    if ((now - last_det_stamp_).seconds() > lost_timeout_sec_)
        return;

    tf2::Transform T_est;
    std::array<double, 36> cov;

    if (!smoother_.estimate(now, T_est, cov))
        return;

    geometry_msgs::msg::PoseWithCovarianceStamped pc;
    pc.header.frame_id = ref_frame_;
    // Publish with current time; consumers doing time-sync can use header
    // stamps from message filters.
    pc.header.stamp = now;
    tf2::toMsg(T_est, pc.pose.pose);
    std::copy(cov.begin(), cov.end(), pc.pose.covariance.begin());
    pub_pose_cov_->publish(pc);

    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tfs;
        tfs.header = pc.header;
        tfs.child_frame_id = base_frame_ + "/refined";
        tfs.transform = tf2::toMsg(T_est);
        tf_broadcaster_->sendTransform(tfs);
    }

    last_pub_stamp_ = now;
}

}  // namespace valve_egomotion

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<valve_egomotion::ValveEgomotionNode>());
    rclcpp::shutdown();
    return 0;
}