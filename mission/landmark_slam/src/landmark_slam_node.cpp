#include <spdlog/spdlog.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_track_array.hpp>

#include "landmark_slam/association.hpp"
#include "landmark_slam/config.hpp"
#include "landmark_slam/graph.hpp"
#include "landmark_slam/targets.hpp"

namespace vortex::landmark_slam {

namespace {

constexpr double kMaxCorrectionSpeed = 0.2;        // map -> odom step [m/s]
constexpr double kMaxCorrectionYawRate = 0.2;      // map -> odom step [rad/s]
constexpr double kNoOrientationVariance = 1000.0;  // perception convention
constexpr std::size_t kNisWindow = 50;
constexpr double kMinRangeM = 0.05;
constexpr double kFramePeriodS = 0.1;  // landmark and gate TF frames
// Detection messages per source and keyframe that update the map (the
// newest; only the last one votes for new landmarks).
constexpr std::size_t kMaxMessagesPerKeyframe = 5;
// A landmark seen fewer times than this and not for kForgetS is hidden: a
// phantom or a bad view, not an object.
constexpr int kMinSupport = 30;
constexpr double kForgetS = 30.0;

gtsam::Pose3 to_pose3(const geometry_msgs::msg::Pose& p) {
    return {gtsam::Rot3::Quaternion(p.orientation.w, p.orientation.x,
                                    p.orientation.y, p.orientation.z),
            gtsam::Point3(p.position.x, p.position.y, p.position.z)};
}

gtsam::Pose3 to_pose3(const geometry_msgs::msg::Transform& t) {
    return {gtsam::Rot3::Quaternion(t.rotation.w, t.rotation.x, t.rotation.y,
                                    t.rotation.z),
            gtsam::Point3(t.translation.x, t.translation.y, t.translation.z)};
}

geometry_msgs::msg::Pose to_msg(const gtsam::Pose3& T) {
    geometry_msgs::msg::Pose p;
    p.position.x = T.x();
    p.position.y = T.y();
    p.position.z = T.z();
    const gtsam::Quaternion q = T.rotation().toQuaternion();
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    return p;
}

builtin_interfaces::msg::Time to_stamp(double t) {
    return rclcpp::Time(static_cast<int64_t>(t * 1e9));
}

/// ROS covariance (x, y, z, then rotation; map axes): the position
/// relative to the vehicle, the rotation from the landmark's marginal.
std::array<double, 36> to_ros_cov(const LandmarkState& l) {
    const gtsam::Matrix3 R = l.pose.rotation().matrix();
    gtsam::Matrix6 out = gtsam::Matrix6::Zero();
    out.block<3, 3>(0, 0) = l.relative_cov;
    out.block<3, 3>(3, 3) = R * l.cov.block<3, 3>(0, 0) * R.transpose();
    std::array<double, 36> a{};
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            a[r * 6 + c] = out(r, c);
        }
    }
    return a;
}

std::string read_text(const std::string& path) {
    std::ifstream f(path);
    std::stringstream text;
    text << f.rdbuf();
    return text.str();
}

/// Check a prior map sent as text and keep it on this machine
/// ($ROS_HOME/landmark_slam/prior_map.yaml, the old one kept with a time
/// stamp). Returns its path; throws with the reason on a bad map.
std::string save_prior_map(const std::string& text, Params params) {
    namespace fs = std::filesystem;
    const char* ros_home = std::getenv("ROS_HOME");
    const char* home = std::getenv("HOME");
    const fs::path dir =
        (ros_home ? fs::path(ros_home) : fs::path(home ? home : ".") / ".ros") /
        "landmark_slam";
    fs::create_directories(dir);
    const fs::path path = dir / "prior_map.yaml";
    const fs::path next = dir / "prior_map.yaml.new";
    std::ofstream(next) << text;
    params.prior_map_file = next.string();
    try {
        load_config(params);
    } catch (const std::exception&) {
        fs::remove(next);
        throw;
    }
    if (fs::exists(path)) {
        const std::time_t now = std::time(nullptr);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S",
                      std::localtime(&now));
        fs::rename(path, dir / ("prior_map_" + std::string(stamp) + ".yaml"));
    }
    fs::rename(next, path);
    return path.string();
}

}  // namespace

class LandmarkSlamNode : public rclcpp::Node {
   public:
    explicit LandmarkSlamNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("landmark_slam_node", options) {
        declare_params();
        cfg_ = load_config(params_);  // fails loudly on a bad file
        // The prior map as text: what prior_map_gui.py gets and sends.
        declare_parameter<std::string>(
            "prior_map",
            params_.use_prior_map && !params_.prior_map_file.empty()
                ? read_text(params_.prior_map_file)
                : "");

        std::string prefix = declare_parameter<std::string>("frame_prefix", "");
        if (!prefix.empty() && prefix.back() == '/') {
            prefix.pop_back();
        }
        frame_prefix_ = prefix.empty() ? "" : prefix + "/";
        map_frame_ = frame_prefix_ + "map";

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        namespace qos = vortex::utils::qos_profiles;
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            declare_parameter<std::string>("topics.odom"),
            qos::sensor_data_profile(10),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                on_odom(*msg);
            });
        detection_sub_ = create_subscription<vortex_msgs::msg::LandmarkArray>(
            declare_parameter<std::string>("topics.landmarks"),
            qos::sensor_data_profile(10),
            [this](vortex_msgs::msg::LandmarkArray::ConstSharedPtr msg) {
                on_detections(msg);
            });
        wipe_sub_ = create_subscription<std_msgs::msg::Empty>(
            declare_parameter<std::string>("topics.mission_wipe"),
            qos::reliable_profile(1),
            [this](std_msgs::msg::Empty::ConstSharedPtr) { on_wipe(); });

        landmarks_pub_ = create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            "landmark_slam/landmarks",
            qos::reliable_transient_local_profile(1));
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "landmark_slam/markers", qos::reliable_transient_local_profile(1));
        nis_pub_ = create_publisher<std_msgs::msg::Float64>(
            "landmark_slam/nis", qos::reliable_profile(10));

        // A new prior_map (the file's text, scripts/prior_map_gui.py) counts
        // from the next mission/wipe.
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                return on_set_parameters(params);
            });

        spdlog::info(
            "landmark_slam: {} classes, {} prior landmarks ({}), map frame "
            "'{}'",
            cfg_.classes.size(), cfg_.prior_landmarks.size(),
            cfg_.initial_pose ? "prior map" : "map = odom at start",
            map_frame_);
    }

   private:
    void declare_params() {
        Params& p = params_;
        p.use_prior_map = declare_parameter<bool>("use_prior_map");
        p.prior_map_file = declare_parameter<std::string>("prior_map_file");
        p.classes_file = declare_parameter<std::string>("classes_file");
        p.keyframe_dist_m = declare_parameter<double>("keyframe_dist_m");
        p.keyframe_time_s = declare_parameter<double>("keyframe_time_s");
        p.odom_sigma_trans_per_m =
            declare_parameter<double>("odom_sigma_trans_per_m");
        p.odom_sigma_yaw_per_m =
            declare_parameter<double>("odom_sigma_yaw_per_m");
        p.default_prior_sigma_xy =
            declare_parameter<double>("default_prior_sigma_xy");
        p.gate_prob = declare_parameter<double>("gate_prob");
        p.min_votes = declare_parameter<double>("min_votes");
        p.vote_radius_m = declare_parameter<double>("vote_radius_m");
        p.bearing_sigma = declare_parameter<double>("bearing_sigma");
        p.range_sigma_a = declare_parameter<double>("range_sigma_a");
        p.range_sigma_b = declare_parameter<double>("range_sigma_b");
        p.gate.panel_classes =
            declare_parameter<std::vector<std::string>>("gate.panel_classes");
        p.gate.min_separation_m =
            declare_parameter<double>("gate.min_separation_m");
        p.gate.max_separation_m =
            declare_parameter<double>("gate.max_separation_m");
        p.gate.depth_below_panel_m =
            declare_parameter<double>("gate.depth_below_panel_m");
        p.gate.approach_m = declare_parameter<double>("gate.approach_m");
    }

    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
            if (p.get_name() == "prior_map") {
                // Checked and saved now, used from the next mission/wipe
                // (the map is rebuilt with it then).
                if (!params_.use_prior_map) {
                    result.successful = false;
                    result.reason = "use_prior_map is false";
                    return result;
                }
                try {
                    params_.prior_map_file =
                        save_prior_map(p.as_string(), params_);
                } catch (const std::exception& e) {
                    result.successful = false;
                    result.reason = e.what();
                    return result;
                }
                spdlog::info(
                    "landmark_slam: new prior map, used from the next "
                    "mission/wipe; to keep it after a restart: "
                    "prior_map_file:={}",
                    params_.prior_map_file);
            }
        }
        return result;
    }

    void reset(const gtsam::Pose3& T_odom_base, double t) {
        graph_.reset(cfg_, T_odom_base, t);
        votes_.clear();
        pending_.clear();
        nis_.clear();
        next_id_ = 1;
        T_map_odom_pub_ = graph_.map_to_odom();
        publish_map(t);
    }

    void on_wipe() {
        try {
            cfg_ = load_config(params_);
        } catch (const std::exception& e) {
            spdlog::error(
                "landmark_slam: reload failed, keeping the old "
                "config: {}",
                e.what());
        }
        if (!odom_) {
            return;  // reset happens with the first odometry
        }
        reset(odom_->T, odom_->t);
        spdlog::info("landmark_slam: reset (mission/wipe)");
    }

    void on_odom(const nav_msgs::msg::Odometry& msg) {
        const double t = rclcpp::Time(msg.header.stamp).seconds();
        const gtsam::Pose3 T = to_pose3(msg.pose.pose);
        odom_frame_ = msg.header.frame_id;
        const bool first = !odom_;
        const double dt_pub = first ? 0.0 : t - odom_->t;
        odom_ = OdomSample{T, t};
        if (first) {
            reset(T, t);
        } else {
            const int kf = graph_.last_keyframe();
            const double dist =
                (graph_.keyframe_odom(kf).translation() - T.translation())
                    .norm();
            const double dt = t - graph_.keyframe_time(kf);
            if (dist >= params_.keyframe_dist_m ||
                dt >= params_.keyframe_time_s) {
                process_keyframe(T, t);
            }
        }
        publish_tf(t, dt_pub);
    }

    void on_detections(
        const vortex_msgs::msg::LandmarkArray::ConstSharedPtr& msg) {
        if (msg->landmarks.empty()) {
            return;
        }
        // Every message since the last keyframe waits for the next, per
        // source (frame and the types it carries).
        std::string source = msg->header.frame_id;
        std::set<std::uint16_t> types;
        for (const auto& l : msg->landmarks) {
            types.insert(l.type.value);
        }
        for (const auto type : types) {
            source += "/" + std::to_string(type);
        }
        auto& q = pending_[source];
        q.push_back(msg);
        if (q.size() > kMaxMessagesPerKeyframe) {
            q.pop_front();
        }
    }

    void process_keyframe(const gtsam::Pose3& T_odom_base, double t) {
        const int kf = graph_.add_keyframe(T_odom_base, t);
        graph_.update();

        // Each message is associated on its own; the detections matched to
        // one landmark become one factor (their mean, Measurement::merged).
        std::map<int, std::pair<Measurement, gtsam::Point3>> matched;
        double t_obs = 0.0;
        const gtsam::Pose3 T_map_base = graph_.keyframe_pose(kf);
        for (const auto& [source, msgs] : pending_) {
            for (const auto& msg : msgs) {
                // Votes for new landmarks only from the newest message: more
                // looks at one object must not make a misclassification a
                // landmark sooner.
                const bool newest = msg == msgs.back();
                const auto detections = to_detections(*msg, T_odom_base);
                if (detections.empty()) {
                    continue;
                }
                const double t_det = rclcpp::Time(msg->header.stamp).seconds();
                const Association a = associate(graph_, kf, detections);
                for (const Match& m : a.matches) {
                    const Measurement& z = detections[m.detection].z;
                    auto [it, first] = matched.try_emplace(
                        m.landmark, z, gtsam::Point3::Zero());
                    auto& [agg, sum] = it->second;
                    if (first) {
                        agg.merged = 0;
                    }
                    sum += z.position;
                    agg.merged++;
                    agg.position = sum / static_cast<double>(agg.merged);
                    if (z.rotation) {
                        agg.rotation = z.rotation;
                    }
                    push_nis(m.nis);
                    t_obs = std::max(t_obs, t_det);
                }
                for (const std::size_t i : a.unmatched) {
                    if (newest) {
                        votes_.add(detections[i], kf, T_map_base, t_det, cfg_);
                    }
                }
            }
        }
        for (const auto& [id, agg] : matched) {
            graph_.add_observation(kf, id, agg.first, t_obs);
        }
        const bool observed = !matched.empty();
        pending_.clear();
        if (observed) {
            graph_.update();
        }

        for (const VoteCluster& c : votes_.take_clusters(graph_, t)) {
            const int id = next_id_++;
            graph_.add_landmark(
                id, c.cls,
                gtsam::Pose3(gtsam::Rot3::Yaw(c.yaw.value_or(0.0)), c.position),
                c.votes.back().t);
            for (const Vote& v : c.votes) {
                graph_.add_observation(v.kf, id, v.z, v.t);
            }
            graph_.update();
            spdlog::info(
                "landmark_slam: new landmark {} ({}) at [{:.2f}, "
                "{:.2f}, {:.2f}] from {} votes",
                id, c.cls.name, c.position.x(), c.position.y(), c.position.z(),
                c.votes.size());
        }
        // Two landmarks of a class closer than vote_radius_m cannot be two
        // objects: the one seen less is dropped once their positions agree.
        const auto dropped =
            graph_.drop_duplicates(cfg_.params.vote_radius_m,
                                   chi2_threshold(cfg_.params.gate_prob, 3));
        for (const auto& [keep, drop] : dropped) {
            spdlog::info("landmark_slam: landmark {} is a duplicate of {}",
                         drop, keep);
        }
        if (!dropped.empty()) {
            graph_.update();  // refresh landmarks() without them
        }
        publish_map(t);
    }

    /// Detections of one message in the base frame of the new keyframe.
    std::vector<Detection> to_detections(
        const vortex_msgs::msg::LandmarkArray& msg,
        const gtsam::Pose3& T_odom_base) {
        // Detection frame -> odom at the image time; odom -> keyframe base.
        gtsam::Pose3 T_odom_frame;
        if (msg.header.frame_id != odom_frame_) {
            try {
                T_odom_frame = to_pose3(
                    tf_buffer_
                        ->lookupTransform(odom_frame_, msg.header.frame_id,
                                          msg.header.stamp)
                        .transform);
            } catch (const tf2::TransformException& e) {
                spdlog::warn("landmark_slam: no TF {} -> {}: {}",
                             msg.header.frame_id, odom_frame_, e.what());
                return {};
            }
        }
        const gtsam::Pose3 T_base_frame = T_odom_base.inverse() * T_odom_frame;

        std::vector<Detection> out;
        for (const auto& l : msg.landmarks) {
            const ClassConfig* cls =
                cfg_.find_class(l.type.value, l.subtype.value);
            if (cls == nullptr) {
                if (unknown_.insert({l.type.value, l.subtype.value}).second) {
                    spdlog::warn(
                        "landmark_slam: type {} subtype {} is not in the "
                        "class file, ignored",
                        l.type.value, l.subtype.value);
                }
                continue;
            }
            const gtsam::Pose3 T_base_obj =
                T_base_frame * to_pose3(l.pose.pose);
            if (T_base_obj.translation().norm() < kMinRangeM) {
                continue;
            }
            Detection d;
            d.cls = cls;
            d.z.position = T_base_obj.translation();
            const auto& c = l.pose.covariance;
            if (c[21] < kNoOrientationVariance &&
                c[28] < kNoOrientationVariance &&
                c[35] < kNoOrientationVariance) {
                d.z.rotation = T_base_obj.rotation();
            }
            out.push_back(d);
        }
        return out;
    }

    void push_nis(double nis) {
        nis_.push_back(nis);
        if (nis_.size() > kNisWindow) {
            nis_.pop_front();
        }
    }

    void publish_tf(double t, double dt) {
        // Move the published correction toward the graph's at a limited
        // rate, so the controller never sees a jump.
        const gtsam::Pose3 target = graph_.map_to_odom();
        const gtsam::Vector6 xi =
            gtsam::Pose3::Logmap(T_map_odom_pub_.between(target));
        double scale = 1.0;
        const double rot = xi.head<3>().norm();
        const double trans = xi.tail<3>().norm();
        if (rot > 0.0) {
            scale = std::min(scale, kMaxCorrectionYawRate * dt / rot);
        }
        if (trans > 0.0) {
            scale = std::min(scale, kMaxCorrectionSpeed * dt / trans);
        }
        T_map_odom_pub_ =
            T_map_odom_pub_ * gtsam::Pose3::Expmap(std::max(scale, 0.0) * xi);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = to_stamp(t);
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = odom_frame_;
        const auto p = to_msg(T_map_odom_pub_);
        tf.transform.translation.x = p.position.x;
        tf.transform.translation.y = p.position.y;
        tf.transform.translation.z = p.position.z;
        tf.transform.rotation = p.orientation;
        tf_broadcaster_->sendTransform(tf);

        // Landmarks and gate frames, with the same stamp: a lookup from odom
        // gets them where the drifted vehicle needs them.
        if (t - last_frames_t_ >= kFramePeriodS || t < last_frames_t_) {
            for (auto& f : frames_) {
                f.header.stamp = tf.header.stamp;
            }
            tf_broadcaster_->sendTransform(frames_);
            last_frames_t_ = t;
        }
    }

    geometry_msgs::msg::TransformStamped frame(const std::string& name,
                                               const gtsam::Pose3& pose) const {
        geometry_msgs::msg::TransformStamped f;
        f.header.frame_id = map_frame_;
        f.child_frame_id = frame_prefix_ + name;
        const auto p = to_msg(pose);
        f.transform.translation.x = p.position.x;
        f.transform.translation.y = p.position.y;
        f.transform.translation.z = p.position.z;
        f.transform.rotation = p.orientation;
        return f;
    }

    void publish_map(double t) {
        graph_.update_covariances();
        vortex_msgs::msg::LandmarkTrackArray array;
        array.header.stamp = to_stamp(t);
        array.header.frame_id = map_frame_;
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);

        frames_.clear();
        // Where the run started (return home), and per class the landmark
        // seen most often (a real object is seen far more often than a
        // phantom or a confused detection): a target for the tree before it
        // knows the id.
        frames_.push_back(frame("start", graph_.keyframe_pose(0)));
        // Where each class should be (prior map): the mean of its entries,
        // at depth 0 and along the map's x axis. A search point for the tree
        // before the object is seen; the depth comes with the offset.
        std::map<std::string, std::pair<gtsam::Point3, int>> prior;
        for (const PriorLandmark& pl : cfg_.prior_landmarks) {
            auto& [sum, n] =
                prior.try_emplace(pl.class_name, gtsam::Point3::Zero(), 0)
                    .first->second;
            sum += gtsam::Point3(pl.x, pl.y, 0.0);
            ++n;
        }
        for (const auto& [cls, sum_n] : prior) {
            frames_.push_back(
                frame("prior_" + cls,
                      gtsam::Pose3(gtsam::Rot3(), sum_n.first / sum_n.second)));
        }
        std::vector<LandmarkState> shown;
        for (const LandmarkState& l : graph_.landmarks()) {
            if (l.n_obs == 0 || l.n_obs >= kMinSupport ||
                t - l.last_seen < kForgetS) {
                shown.push_back(l);
            }
        }
        std::map<std::string, const LandmarkState*> best;
        for (const LandmarkState& l : shown) {
            const LandmarkState*& b = best[l.cls.name];
            if (!b || l.n_obs > b->n_obs) {
                b = &l;
            }
        }
        for (const auto& [cls, l] : best) {
            frames_.push_back(frame(cls, l->pose));
        }
        for (const TargetFrame& g :
             gate_frames(shown, cfg_.params.gate,
                         graph_.keyframe_pose(0).translation())) {
            frames_.push_back(frame(g.name, g.pose));
        }
        for (const LandmarkState& l : shown) {
            frames_.push_back(
                frame(fmt::format("{}_{}", l.cls.name, l.id), l.pose));
            vortex_msgs::msg::LandmarkTrack track;
            track.header = array.header;
            track.landmark.header = array.header;
            track.landmark.id = l.id;
            track.landmark.type.value = l.cls.type;
            track.landmark.subtype.value = l.cls.subtype;
            track.landmark.pose.pose = to_msg(l.pose);
            track.landmark.pose.covariance = to_ros_cov(l);
            track.confirmed = l.n_obs > 0;
            track.has_orientation = l.yaw_known;
            track.observations = l.n_obs;
            track.first_seen = to_stamp(l.first_seen);
            track.last_measurement = to_stamp(l.last_seen);
            array.landmark_tracks.push_back(track);
            add_markers(l, track, markers);
        }
        landmarks_pub_->publish(array);
        markers_pub_->publish(markers);

        if (!nis_.empty()) {
            std_msgs::msg::Float64 nis;
            nis.data = std::accumulate(nis_.begin(), nis_.end(), 0.0) /
                       static_cast<double>(nis_.size());
            nis_pub_->publish(nis);
            if (nis_.size() == kNisWindow &&
                (nis.data < 0.5 || nis.data > 2.0) &&
                t - last_nis_warn_ > 10.0) {
                spdlog::warn(
                    "landmark_slam: mean NIS {:.2f} over the last {} matches "
                    "(expect ~1): the noise values are {}; recalibrate",
                    nis.data, kNisWindow,
                    nis.data > 1.0 ? "too optimistic" : "too pessimistic");
                last_nis_warn_ = t;
            }
        }
    }

    /// Sphere at 2 sigma of the position covariance, and a label.
    void add_markers(const LandmarkState& l,
                     const vortex_msgs::msg::LandmarkTrack& track,
                     visualization_msgs::msg::MarkerArray& out) const {
        const auto& c = track.landmark.pose.covariance;
        Eigen::Matrix3d P;
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) {
                P(r, k) = c[r * 6 + k];
            }
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(P);
        Eigen::Matrix3d axes = eig.eigenvectors();
        if (axes.determinant() < 0.0) {
            axes.col(2) *= -1.0;
        }
        const Eigen::Quaterniond q(axes);
        const Eigen::Vector3d sd = eig.eigenvalues().cwiseMax(0.0).cwiseSqrt();

        visualization_msgs::msg::Marker m;
        m.header = track.header;
        m.ns = "landmarks";
        m.id = l.id;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.pose.position = track.landmark.pose.pose.position;
        m.pose.orientation.w = q.w();
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.scale.x = std::max(0.05, 2.0 * sd(0));
        m.scale.y = std::max(0.05, 2.0 * sd(1));
        m.scale.z = std::max(0.05, 2.0 * sd(2));
        m.color.a = 0.6;
        m.color.r = l.n_obs > 0 ? 0.1 : 0.6;
        m.color.g = l.n_obs > 0 ? 0.8 : 0.6;
        m.color.b = l.n_obs > 0 ? 0.2 : 0.6;
        out.markers.push_back(m);

        m.ns = "labels";
        m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        m.pose.orientation = geometry_msgs::msg::Quaternion();
        m.pose.position.z -= 0.3;
        m.scale.x = m.scale.y = m.scale.z = 0.2;
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0;
        m.text = fmt::format("{} {} n={}", l.id, l.cls.name, l.n_obs);
        out.markers.push_back(m);
    }

    struct OdomSample {
        gtsam::Pose3 T;
        double t{0.0};
    };

    Params params_;
    Config cfg_;
    LandmarkGraph graph_;
    Votes votes_;
    /// Per source, the latest kMaxMessagesPerKeyframe messages.
    std::map<std::string,
             std::deque<vortex_msgs::msg::LandmarkArray::ConstSharedPtr>>
        pending_;
    std::optional<OdomSample> odom_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string frame_prefix_;
    std::vector<geometry_msgs::msg::TransformStamped> frames_;
    double last_frames_t_{0.0};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    gtsam::Pose3 T_map_odom_pub_;
    int next_id_{1};
    std::deque<double> nis_;
    double last_nis_warn_{0.0};
    std::set<std::pair<std::uint16_t, std::uint16_t>> unknown_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
        detection_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr wipe_sub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        landmarks_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_pub_;
};

}  // namespace vortex::landmark_slam

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::landmark_slam::LandmarkSlamNode)
