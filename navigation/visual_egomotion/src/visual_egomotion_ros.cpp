#include "visual_egomotion/visual_egomotion_ros.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <limits>

namespace visual_egomotion {

VisualEgomotionNode::VisualEgomotionNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("visual_egomotion", options)
{
    set_subscribers_and_publisher();
    set_parameters();

    SlidingWindowSO3Mean::Config cfg;
    cfg.win_size = this->get_parameter("window_size").as_int();
    cfg.win_age_sec = this->get_parameter("max_age_sec").as_double();
    cfg.huber_threshold_deg = this->get_parameter("huber_threshold_deg").as_double();
    cfg.gate_threshold_deg = this->get_parameter("gate_threshold_deg").as_double();
    cfg.use_mad_gate = this->get_parameter("use_mad").as_bool();
    smoother_ = std::make_unique<SlidingWindowSO3Mean>(cfg);

    last_det_stamp = this->now();
    last_pub_stamp = this->now();
    last_pub_det_stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(1.0, pub_rate_hz_));
    pub_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&VisualEgomotionNode::onPublishTimer, this));

    RCLCPP_INFO(
        this->get_logger(),
        "VisualEgomotionNode online. Sub: %s, base: %s, cam: %s, ref: %s",
        visual_topic_.c_str(), base_frame_.c_str(), cam_frame_.c_str(),
        ref_frame_.c_str());
}

void VisualEgomotionNode::set_subscribers_and_publisher()
{
    this->declare_parameter<std::string>("visual_topic");
    visual_topic_ = this->get_parameter("visual_topic").as_string();

    pub_pose_cov_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/landmark/pose_cov", rclcpp::QoS(10));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_landmarks_ = this->create_subscription<vortex_msgs::msg::LandmarkArray>(
        visual_topic_, rclcpp::SensorDataQoS(),
        std::bind(&VisualEgomotionNode::onLandmarks, this, std::placeholders::_1));
}

void VisualEgomotionNode::set_parameters()
{
    this->declare_parameter<std::string>("ref_frame");
    this->declare_parameter<std::string>("base_frame");
    this->declare_parameter<std::string>("cam_frame");

    this->declare_parameter<bool>("publish_tf");
    this->declare_parameter<double>("pub_rate_hz");
    this->declare_parameter<double>("lost_timeout_sec");

    // smoother params
    this->declare_parameter<int>("window_size");
    this->declare_parameter<double>("max_age_sec");
    this->declare_parameter<double>("huber_threshold_deg");
    this->declare_parameter<double>("gate_threshold_deg");
    this->declare_parameter<bool>("use_mad");

    // get
    ref_frame_ = this->get_parameter("ref_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    cam_frame_ = this->get_parameter("cam_frame").as_string();

    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    pub_rate_hz_ = this->get_parameter("pub_rate_hz").as_double();
    lost_timeout_sec_ = this->get_parameter("lost_timeout_sec").as_double();

}

void VisualEgomotionNode::onLandmarks(
    const vortex_msgs::msg::LandmarkArray::SharedPtr msg) {
    if (!msg || msg->landmarks.empty())
        return;

    const vortex_msgs::msg::Landmark* board = nullptr;
    std::vector<const vortex_msgs::msg::Landmark*> markers;
    markers.reserve(msg->landmarks.size());

    for (const auto& lm : msg->landmarks) {
        if (lm.type == vortex_msgs::msg::Landmark::ARUCO_MARKER) {
            markers.push_back(&lm);
        }
    }

    if (!board && markers.empty())
        return;

    const vortex_msgs::msg::Landmark* chosen = nullptr;
    uint16_t chosen_id = 0xFFFF;

    if (board) {
        chosen = board;
        chosen_id = 0;
    } else {
        auto it = std::min_element(markers.begin(), markers.end(), [](const auto* a, const auto* b) { return a->subtype < b->subtype; });
        chosen = *it;
        chosen_id = chosen->subtype;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Chosen=%s id=%u (anchored=%d) markers=%zu",
                         (chosen == board) ? "BOARD" : "MARKER", chosen_id,
                         last_used_marker_id_, markers.size());

    tf2::Transform T_cam_landmark;
    tf2::fromMsg(chosen->pose.pose, T_cam_landmark);

    tf2::Transform T_base_cam;
    try {
        const auto ts = tf_buffer->lookupTransform(base_frame_, cam_frame_,
                                                    tf2::TimePointZero);
        tf2::fromMsg(ts.transform, T_base_cam);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for TF %s <- %s: %s", base_frame_.c_str(),
                             cam_frame_.c_str(), ex.what());
        return;
    }

    const tf2::Transform T_base_landmark_now = T_base_cam * T_cam_landmark;

    if (have_ref && chosen != board && last_used_marker_id_ != 0xFFFF &&
        chosen_id != last_used_marker_id_) {
        tf2::Transform T_ref_base_curr;
        std::array<double, 36> tmp_cov;

        if (smoother_->estimate(this->now(), T_ref_base_curr, tmp_cov)) {
            last_T_ref_base_ = T_ref_base_curr;
            have_last_ref_base_ = true;
        } else if (have_last_ref_base_) {
            T_ref_base_curr = last_T_ref_base_;
        } else {
            T_ref_base_curr =
                T_base_marker0_ * T_base_landmark_now.inverse();
            last_T_ref_base_ = T_ref_base_curr;
            have_last_ref_base_ = true;
        }

        // Re-anchor so T_ref<-base stays the same under the new marker
        T_base_marker0_ = T_ref_base_curr * T_base_landmark_now;

        RCLCPP_WARN(this->get_logger(),
                    "Marker switch %u -> %u: re-anchored to preserve continuity",
                    last_used_marker_id_, chosen_id);
    }

    if (!have_ref) {
        T_base_marker0_ = T_base_landmark_now;
        have_ref = true;

        if (chosen != board) {
            last_used_marker_id_ = chosen_id;  
        }

        if (chosen == board) {
            RCLCPP_INFO(this->get_logger(), "Anchor set to ARUCO_BOARD");
        } else {
            RCLCPP_INFO(this->get_logger(), "Anchor set to ARUCO_MARKER id=%u",
                        chosen_id);
        }
    } else {
        if (chosen != board) {
            const bool changed = (last_used_marker_id_ != 0xFFFF) &&
                                 (chosen_id != last_used_marker_id_);
            if (changed) {
                RCLCPP_INFO(this->get_logger(), "Using marker id=%u (prev=%u)",
                            chosen_id, last_used_marker_id_);
            }
            last_used_marker_id_ = chosen_id;
        }
    }

    const tf2::Transform T_ref_base_meas =
        T_base_marker0_ * T_base_landmark_now.inverse();

    last_T_ref_base_ = T_ref_base_meas;
    have_last_ref_base_ = true;

    smoother_->addSample(msg->header.stamp, T_ref_base_meas, 1.0);
    smoother_->prune(this->now());
    last_det_stamp = msg->header.stamp;
}

void VisualEgomotionNode::onPublishTimer() {
    const rclcpp::Time now = this->now();

    if (!have_ref)
        return;
    //if ((now - last_det_stamp).seconds() > lost_timeout_sec_)
    if ((now - last_det_stamp).seconds() > lost_timeout_sec_) {
        have_ref = false;
        return;
    }
    //    return;

    if (last_det_stamp == last_pub_det_stamp) return;

    tf2::Transform T_est;
    std::array<double, 36> cov;

    if (!smoother_->estimate(now, T_est, cov))
        return;

    geometry_msgs::msg::PoseWithCovarianceStamped pc;
    pc.header.frame_id = ref_frame_;
    pc.header.stamp = last_det_stamp;
    tf2::toMsg(T_est, pc.pose.pose);
    std::copy(cov.begin(), cov.end(), pc.pose.covariance.begin());
    pub_pose_cov_->publish(pc);

    last_pub_det_stamp = last_det_stamp;
    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tfs;
        tfs.header = pc.header;
        tfs.child_frame_id = base_frame_ + "/refined";
        tfs.transform = tf2::toMsg(T_est);
        tf_broadcaster_->sendTransform(tfs);
    }

    last_pub_stamp = now;
}

}  // namespace visual_egomotion

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<visual_egomotion::VisualEgomotionNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}