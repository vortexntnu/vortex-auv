#include "landmark_server/landmark_server_ros.hpp"

#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_transforms.hpp>

namespace vortex::mission {

namespace {

const auto start_msg = R"(
██       █████  ███    ██ ██████  ███    ███  █████  ██████  ██   ██     ███████ ███████ ██████  ██    ██ ███████ ██████
██      ██   ██ ████   ██ ██   ██ ████  ████ ██   ██ ██   ██ ██  ██      ██      ██      ██   ██ ██    ██ ██      ██   ██
██      ███████ ██ ██  ██ ██   ██ ██ ████ ██ ███████ ██████  █████       ███████ █████   ██████  ██    ██ █████   ██████
██      ██   ██ ██  ██ ██ ██   ██ ██  ██  ██ ██   ██ ██   ██ ██  ██           ██ ██      ██   ██  ██  ██  ██      ██   ██
███████ ██   ██ ██   ████ ██████  ██      ██ ██   ██ ██   ██ ██   ██     ███████ ███████ ██   ██   ████   ███████ ██   ██
)";

}  // namespace

LandmarkServerNode::LandmarkServerNode(const rclcpp::NodeOptions& options)
    // Undeclared parameters are allowed so that a rule that is not in the
    // config files can be set live (on_parameters_set validates it).
    : rclcpp::Node(
          "landmark_server_node",
          rclcpp::NodeOptions(options).allow_undeclared_parameters(true)) {
    timer_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    target_frame_ = this->declare_parameter<std::string>("target_frame");

    create_track_manager();
    load_config();
    create_outputs();
    create_debug_outputs();
    create_polling_action_server();
    create_detection_subscription();
    create_odom_subscription();
    create_reset_subscription();
    create_timer();
    spdlog::info(start_msg);
}

void LandmarkServerNode::create_track_manager() {
    // The default class settings; per-class ones (track_config.<CLASS>) and
    // the track limits come in load_config.
    auto& d = track_manager_config_.default_class_config;
    const std::string p = "track_config.default.";
    d.nm.confirm_n = this->declare_parameter<int>(p + "nm.confirm_n", 3);
    d.nm.confirm_m = this->declare_parameter<int>(p + "nm.confirm_m", 5);
    d.nm.delete_n = this->declare_parameter<int>(p + "nm.delete_n", 5);
    d.nm.delete_m = this->declare_parameter<int>(p + "nm.delete_m", 7);
    d.min_pos_error =
        this->declare_parameter<double>(p + "gate.min_pos_error", 0.0);
    d.max_pos_error =
        this->declare_parameter<double>(p + "gate.max_pos_error", 1.5);
    d.min_ori_error =
        this->declare_parameter<double>(p + "gate.min_ori_error", 0.0);
    d.max_ori_error =
        this->declare_parameter<double>(p + "gate.max_ori_error", 0.5);
    d.dyn_std_dev = this->declare_parameter<double>(p + "dyn_mod_std_dev", 0.2);
    d.sens_std_dev =
        this->declare_parameter<double>(p + "sens_mod_std_dev", 0.2);
    d.init_pos_std =
        this->declare_parameter<double>(p + "init_pos_std_dev", 0.1);
    d.init_ori_std =
        this->declare_parameter<double>(p + "init_ori_std_dev", 0.05);
    d.mahalanobis_threshold =
        this->declare_parameter<double>(p + "mahalanobis_gate_threshold", 3.4);
    d.prob_of_detection =
        this->declare_parameter<double>(p + "prob_of_detection", 1.0);
    d.clutter_intensity =
        this->declare_parameter<double>(p + "clutter_intensity", 0.0);
    track_manager_ = std::make_unique<vortex::filtering::PoseTrackManager>(
        track_manager_config_);
}

void LandmarkServerNode::create_detection_subscription() {
    // Queue of 10 so bursts and several detectors do not drop messages.
    const auto qos = vortex::utils::qos_profiles::sensor_data_profile(10);
    auto sub = std::make_shared<
        message_filters::Subscriber<vortex_msgs::msg::LandmarkArray>>(
        this, this->declare_parameter<std::string>("topics.landmarks"),
        qos.get_rmw_qos_profile());

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface()));
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Each detection message waits until the TF from its camera frame to
    // target_frame_ at its stamp (the image time) is there.
    auto filter = std::make_shared<
        tf2_ros::MessageFilter<vortex_msgs::msg::LandmarkArray>>(
        *sub, *tf2_buffer_, target_frame_, 10,
        this->get_node_logging_interface(), this->get_node_clock_interface());
    filter->registerCallback(
        [this](const vortex_msgs::msg::LandmarkArray::ConstSharedPtr msg) {
            vortex_msgs::msg::LandmarkArray in_target;
            try {
                vortex::utils::ros_transforms::transform_pose(
                    *tf2_buffer_, *msg, target_frame_, in_target);
            } catch (const tf2::TransformException& ex) {
                spdlog::warn("TF transform failed from '{}' to '{}': {}",
                             msg->header.frame_id, target_frame_, ex.what());
                return;
            }
            auto landmarks = ros_msg_to_landmarks(in_target);
            std::lock_guard<std::mutex> lock(measurements_mtx_);
            measurements_.insert(measurements_.end(),
                                 std::make_move_iterator(landmarks.begin()),
                                 std::make_move_iterator(landmarks.end()));
            ++frames_received_;
        });
    landmark_sub_ = sub;
    tf_filter_ = filter;
}

void LandmarkServerNode::create_odom_subscription() {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->declare_parameter<std::string>("topics.odom"),
        vortex::utils::qos_profiles::sensor_data_profile(1),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            const auto pose = odom_in_target_frame(*msg);
            std::lock_guard<std::mutex> lock(odom_mtx_);
            last_odom_position_ = msg->pose.pose.position;
            if (pose) {
                last_odom_pose_.emplace(
                    rclcpp::Time(msg->header.stamp).seconds(), *pose);
            }
        });
}

void LandmarkServerNode::create_reset_subscription() {
    rclcpp::SubscriptionOptions options;
    options.callback_group = timer_cb_group_;
    reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "mission/wipe", vortex::utils::qos_profiles::reliable_profile(1),
        [this](std_msgs::msg::Empty::ConstSharedPtr) { on_system_reset(); },
        options);
}

void LandmarkServerNode::create_timer() {
    const int period_ms = this->declare_parameter<int>("timer_rate_ms", 200);
    filter_dt_seconds_ = static_cast<double>(period_ms) / 1000.0;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms), [this] { timer_callback(); },
        timer_cb_group_);
}

void LandmarkServerNode::on_system_reset() {
    abort_polling_goal();
    track_manager_ = std::make_unique<vortex::filtering::PoseTrackManager>(
        track_manager_config_);
    filter_time_sec_.reset();
    reset_map();
    spdlog::info("LandmarkServer: reset complete");
}

void LandmarkServerNode::timer_callback() {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto ms_since = [](Clock::time_point t) {
        return std::chrono::duration<double, std::milli>(Clock::now() - t)
            .count();
    };
    std::vector<Landmark> measurements;
    bool had_frame = false;
    {
        std::lock_guard<std::mutex> lock(measurements_mtx_);
        measurements.swap(measurements_);
        had_frame = frames_received_ != frames_counted_;
        frames_counted_ = frames_received_;
    }
    apply_pending_map_config();
    gate_measurements(measurements);
    step_tracker(std::move(measurements), had_frame);
    const double tracker_ms = ms_since(start);

    const auto dropped = dropped_measurements_.load();
    if (dropped != reported_dropped_measurements_) {
        spdlog::warn(
            "LandmarkServer: {} invalid measurements discarded in total",
            dropped);
        reported_dropped_measurements_ = dropped;
    }

    const auto map_start = Clock::now();
    update_map();
    const double map_ms = ms_since(map_start);
    const auto publish_start = Clock::now();
    publish_map();
    publish_course_frame();
    publish_course_state();
    publish_debug();
    serve_polling_goal();
    const double publish_ms = ms_since(publish_start);

    // Services run between ticks: a tick longer than the period delays them
    // and the next tick. Said at most every 10 s, with the slowest since.
    const double tick_ms = ms_since(start);
    if (tick_ms > filter_dt_seconds_ * 1000.0) {
        slowest_tick_ms_ = std::max(slowest_tick_ms_, tick_ms);
        if (Clock::now() - last_slow_tick_log_ > std::chrono::seconds(10)) {
            spdlog::warn(
                "LandmarkServer: a tick took {:.0f} ms (tracker {:.0f}, map "
                "{:.0f}, "
                "publish {:.0f}), the period is {:.0f} ms; slowest since the "
                "last "
                "warning {:.0f} ms",
                tick_ms, tracker_ms, map_ms, publish_ms,
                filter_dt_seconds_ * 1000.0, slowest_tick_ms_);
            last_slow_tick_log_ = Clock::now();
            slowest_tick_ms_ = 0.0;
        }
    }
}

void LandmarkServerNode::step_tracker(std::vector<Landmark> measurements,
                                      bool had_frame) {
    // One tracker update per camera frame (same stamp), in time order, so
    // two frames of the same object are fused one after the other instead
    // of competing in one update. The filters run on measurement time; a
    // tick without frames predicts by the tick period. A frame older than
    // the filter time (camera latency) is applied without predicting
    // further; a jump larger than max_stamp_dt_seconds_ (clock change, bag
    // loop) resynchronises the filter time.
    std::map<double, std::vector<Landmark>> frames;
    for (auto& m : measurements) {
        frames[m.stamp_sec].push_back(std::move(m));
    }
    if (frames.empty()) {
        if (filter_time_sec_) {
            *filter_time_sec_ += filter_dt_seconds_;
        }
        std::vector<Landmark> none;
        track_manager_->update(none, filter_dt_seconds_);
    }
    for (auto& [stamp, frame] : frames) {
        double dt = filter_dt_seconds_;
        if (!filter_time_sec_) {
            filter_time_sec_ = stamp;
        } else {
            const double stamp_dt = stamp - *filter_time_sec_;
            if (std::abs(stamp_dt) > max_stamp_dt_seconds_) {
                spdlog::warn(
                    "LandmarkServer: measurement time jumped {:.3f} s, "
                    "resynchronising",
                    stamp_dt);
                filter_time_sec_ = stamp;
            } else {
                dt = std::max(stamp_dt, min_step_dt_seconds_);
                filter_time_sec_ = std::max(*filter_time_sec_, stamp);
            }
        }
        track_manager_->update(frame, dt);
        const auto& assoc = track_manager_->last_associations();
        tick_associations_.insert(tick_associations_.end(), assoc.begin(),
                                  assoc.end());
    }
    // Hits and misses once per tick, and only when a detection message came
    // in: with a detector slower than the tick, a tick between two camera
    // frames is not a miss.
    if (had_frame) {
        track_manager_->end_cycle();
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandmarkServerNode)

}  // namespace vortex::mission
