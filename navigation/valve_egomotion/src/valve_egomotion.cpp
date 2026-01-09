#include "valve_egomotion/valve_egomotion.hpp"
#include <algorithm>

namespace valve_egomotion {

ValveEgomotionNode::ValveEgomotionNode() : Node("valve_egomotion") {
    std::string valve_topic =
        declare_parameter("valve_topic", "/aruco_detector/markers");
    ref_frame_ = declare_parameter("ref_frame", "vo_ref");
    base_frame_ = declare_parameter("base_frame", "base_link");
    cam_frame_ = declare_parameter("cam_frame", "Orca/camera_front");

    win_size_ = declare_parameter("window_size", 20);
    win_age_sec_ = declare_parameter("max_age_sec", 1.0);
    huber_deg_ = declare_parameter("huber_threshold_deg", 2.5);
    gate_deg_ = declare_parameter("gate_threshold_deg", 15.0);
    use_mad_ = declare_parameter("use_mad", false);
    pub_rate_hz_ = declare_parameter("pub_rate_hz", 30.0);
    publish_tf_ = declare_parameter("publish_tf", false);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_marker_ = create_subscription<geometry_msgs::msg::PoseArray>(
        valve_topic, rclcpp::SensorDataQoS(),
        std::bind(&ValveEgomotionNode::valvePoseCallback, this,
                  std::placeholders::_1));

    pub_pose_cov_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/landmark/pose_cov", 10);

    last_pub_time_ = this->now();
    RCLCPP_INFO(get_logger(), "Valve Estimator Node Online. Watching: %s",
                valve_topic.c_str());
}

void ValveEgomotionNode::valvePoseCallback(
    const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.empty())
        return;

    tf2::Transform T_cam_marker;
    tf2::fromMsg(msg->poses[0], T_cam_marker);

    tf2::Transform T_base_cam;

try {
    auto ts_base_cam = tf_buffer_->lookupTransform(base_frame_, cam_frame_,
                                                   tf2::TimePointZero);
    tf2::fromMsg(ts_base_cam.transform, T_base_cam);
} catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Waiting for TF frames : %s", ex.what());
    return;
}

// base <- marker (measured)
tf2::Transform T_base_marker_now = T_base_cam * T_cam_marker;

// latch reference at first good detection
if (!have_ref_) {
    T_base_marker0_ = T_base_marker_now;
    have_ref_ = true;
}

// ref(base0) <- base(now)
tf2::Transform T_ref_base_meas = T_base_marker0_ * T_base_marker_now.inverse();

buf_.push_back({msg->header.stamp, T_ref_base_meas, 1.0});
    prune(this->now());

    if ((this->now() - last_pub_time_).seconds() < (1.0 / pub_rate_hz_))
        return;
    last_pub_time_ = this->now();

    tf2::Transform T_est;
    double cov[36];

    if (estimateWindow(T_est, cov, this->now())) {
        geometry_msgs::msg::PoseWithCovarianceStamped pc;
        pc.header.frame_id = ref_frame_;
        pc.header.stamp = msg->header.stamp;  // For ESKF time synchronization
        tf2::toMsg(T_est, pc.pose.pose);
        std::copy(std::begin(cov), std::end(cov), pc.pose.covariance.begin());
        pub_pose_cov_->publish(pc);

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tfs;
            tfs.header = pc.header;
            tfs.child_frame_id = base_frame_ + "/refined";
            tfs.transform = tf2::toMsg(T_est);
            tf_broadcaster_->sendTransform(tfs);
        }
    }
}

bool ValveEgomotionNode::estimateWindow(tf2::Transform& T_est,
                                        double* cov6x6,
                                        const rclcpp::Time& nowt) {
    if (buf_.empty())
        return false;

    tf2::Quaternion q_avg = buf_.back().T_ref_base.getRotation();
    tf2::Vector3 p_avg(0, 0, 0);
    double current_gate = gate_deg_ * M_PI / 180.0;
    const double huber = huber_deg_ * M_PI / 180.0;

    if (use_mad_ && buf_.size() >= 5) {
        std::vector<double> dists;
        for (auto& s : buf_)
            dists.push_back(angularDist(q_avg, s.T_ref_base.getRotation()));
        double med = getMedian(dists);
        for (auto& d : dists)
            d = std::abs(d - med);
        current_gate = std::min(current_gate, med + 3.0 * getMedian(dists));
    }

    for (int i = 0; i < 5; ++i) {
        tf2::Vector3 sum_p(0, 0, 0), sum_q(0, 0, 0);
        double w_total = 0;
        for (auto& s : buf_) {
            double dist = angularDist(q_avg, s.T_ref_base.getRotation());
            if (dist > current_gate)
                continue;
            tf2::Vector3 wi =
                logMapSO3(q_avg.inverse() * s.T_ref_base.getRotation());
            double weight = (huber > 0 && wi.length() > huber)
                                ? (huber / wi.length())
                                : 1.0;
            sum_p += s.T_ref_base.getOrigin() * weight;
            sum_q += wi * weight;
            w_total += weight;
        }
        if (w_total < 1e-6)
            return false;
        p_avg = sum_p / w_total;
        tf2::Vector3 dq = sum_q / w_total;
        q_avg = (q_avg * expMapSO3(dq)).normalized();
        if (dq.length() < 1e-4)
            break;
    }
    T_est.setOrigin(p_avg);
    T_est.setRotation(q_avg);

    std::fill(cov6x6, cov6x6 + 36, 0.0);
    double inliers = 0;
    for (auto& s : buf_) {
        if (angularDist(q_avg, s.T_ref_base.getRotation()) > current_gate)
            continue;
        tf2::Vector3 dp = s.T_ref_base.getOrigin() - p_avg;
        tf2::Vector3 dq =
            logMapSO3(q_avg.inverse() * s.T_ref_base.getRotation());

        cov6x6[0] += dp.x() * dp.x();   // x
        cov6x6[7] += dp.y() * dp.y();   // y
        cov6x6[14] += dp.z() * dp.z();  // z
        cov6x6[21] += dq.x() * dq.x();  // roll
        cov6x6[28] += dq.y() * dq.y();  // pitch
        cov6x6[35] += dq.z() * dq.z();  // yaw
        inliers += 1.0;
    }
    for (int i = 0; i < 36; i++)
        cov6x6[i] /= std::max(1.0, inliers);

    const double pos_floor = 1e-6;  // 1mm variance
    const double rot_floor = 1e-6;  // ~0.05 degree variance

    cov6x6[0] += pos_floor;
    cov6x6[7] += pos_floor;
    cov6x6[14] += pos_floor;
    cov6x6[21] += rot_floor;
    cov6x6[28] += rot_floor;
    cov6x6[35] += rot_floor;
    return true;
}

tf2::Vector3 ValveEgomotionNode::logMapSO3(const tf2::Quaternion& q) {
    double w = std::clamp((double)q.getW(), -1.0, 1.0);
    double th = 2.0 * acos(w);
    double s = sqrt(std::max(0.0, 1.0 - w * w));

    if (s < 1e-9)
        return tf2::Vector3(2 * q.getX(), 2 * q.getY(), 2 * q.getZ());

    return tf2::Vector3(q.getX(), q.getY(), q.getZ()) * (th / s);
}
tf2::Quaternion ValveEgomotionNode::expMapSO3(const tf2::Vector3& w) {
    double th = w.length();

    if (th < 1e-9)
        return tf2::Quaternion(w.x() * 0.5, w.y() * 0.5, w.z() * 0.5, 1.0)
            .normalized();
    tf2::Vector3 a = w / th;

    return tf2::Quaternion(a.x() * sin(th * 0.5), a.y() * sin(th * 0.5),
                           a.z() * sin(th * 0.5), cos(th * 0.5));
}
double ValveEgomotionNode::angularDist(const tf2::Quaternion& q1,
                                       const tf2::Quaternion& q2) {
    return 2.0 * acos(std::clamp(std::abs((double)q1.dot(q2)), -1.0, 1.0));
}
template <typename T>
T ValveEgomotionNode::getMedian(std::vector<T> v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}
void ValveEgomotionNode::prune(const rclcpp::Time& nowt) {
    while (!buf_.empty() &&
           (nowt - buf_.front().stamp).seconds() > win_age_sec_)
        buf_.pop_front();

    while ((int)buf_.size() > win_size_)
        buf_.pop_front();
}
}  // namespace valve_egomotion

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<valve_egomotion::ValveEgomotionNode>());
    rclcpp::shutdown();

    return 0;
}