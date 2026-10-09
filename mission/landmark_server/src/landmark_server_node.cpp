#include <spdlog/spdlog.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_track_array.hpp>
#include <vortex_msgs/srv/set_premap.hpp>

#include "landmark_server/association.hpp"
#include "landmark_server/config.hpp"
#include "landmark_server/graph.hpp"
#include "landmark_server/premap.hpp"

namespace vortex::landmark_server {

namespace {

constexpr double kNoOrientationVariance = 1000.0;  // perception convention
constexpr std::size_t kNisWindow = 50;
constexpr double kMinRangeM = 0.05;
constexpr double kFramePeriodS = 0.1;  // landmark TF frames
constexpr const char* kGuiPrefix = "__gui/";
constexpr double kPriorWarnPeriodS = 10.0;
constexpr int kPriorWarnMinCount = 20;

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

/// The most observed landmark of the class, or nullptr.
const LandmarkState* best_of(const std::vector<LandmarkState>& landmarks,
                             const std::string& cls) {
    const LandmarkState* best = nullptr;
    for (const auto& l : landmarks) {
        if (l.cls.name == cls && (!best || l.n_obs > best->n_obs)) {
            best = &l;
        }
    }
    return best;
}

struct NamedPose {
    std::string name;
    gtsam::Pose3 pose;
};

/**
 * Gate frames from the two role panels, when both are mapped and
 * min_separation_m..max_separation_m apart: gate_middle between them, and
 * per panel <panel>_entrance / <panel>_exit approach_m before / after the
 * gate line and depth_below_panel_m below the panel (through its opening).
 * All have +X through the gate, away from the start side.
 */
std::vector<NamedPose> gate_frames(const std::vector<LandmarkState>& landmarks,
                                   const GateParams& gate,
                                   const gtsam::Point3& start) {
    if (gate.panel_classes.size() != 2) {
        return {};
    }
    const LandmarkState* a = best_of(landmarks, gate.panel_classes[0]);
    const LandmarkState* b = best_of(landmarks, gate.panel_classes[1]);
    if (!a || !b) {
        return {};
    }
    const gtsam::Point3 along = b->pose.translation() - a->pose.translation();
    const double separation = std::hypot(along.x(), along.y());
    if (separation < gate.min_separation_m ||
        separation > gate.max_separation_m) {
        return {};
    }
    const gtsam::Point3 middle =
        0.5 * (a->pose.translation() + b->pose.translation());
    double yaw = std::atan2(along.x(), -along.y());
    const gtsam::Point3 to_middle = middle - start;
    if (std::cos(yaw) * to_middle.x() + std::sin(yaw) * to_middle.y() < 0.0) {
        yaw += M_PI;
    }
    const gtsam::Rot3 R = gtsam::Rot3::Yaw(yaw);
    const gtsam::Point3 through(std::cos(yaw), std::sin(yaw), 0.0);
    const gtsam::Point3 down(0.0, 0.0, gate.depth_below_panel_m);
    std::vector<NamedPose> out{{"gate_middle", gtsam::Pose3(R, middle)}};
    for (const LandmarkState* panel : {a, b}) {
        const gtsam::Point3 p = panel->pose.translation() + down;
        out.push_back({panel->cls.name + "_entrance",
                       gtsam::Pose3(R, p - gate.approach_m * through)});
        out.push_back({panel->cls.name + "_exit",
                       gtsam::Pose3(R, p + gate.approach_m * through)});
    }
    return out;
}

}  // namespace

class LandmarkServerNode : public rclcpp::Node {
   public:
    explicit LandmarkServerNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node(
              "landmark_server_node",
              rclcpp::NodeOptions(options)
                  .automatically_declare_parameters_from_overrides(true)) {
        load_config();  // fails loudly on a bad config

        std::string prefix = get_parameter_or<std::string>("frame_prefix", "");
        if (!prefix.empty() && prefix.back() == '/') {
            prefix.pop_back();
        }
        frame_prefix_ = prefix.empty() ? "" : prefix + "/";
        map_frame_ = frame_prefix_ + "map";

        premap_file_ = get_parameter_or<std::string>("premap_file", "");
        load_premap_file();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        namespace qos = vortex::utils::qos_profiles;
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter_or<std::string>("topics.odom", "odom"),
            qos::sensor_data_profile(10),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                on_odom(*msg);
            });
        detection_sub_ = create_subscription<vortex_msgs::msg::LandmarkArray>(
            get_parameter_or<std::string>("topics.landmarks", "landmarks"),
            qos::sensor_data_profile(10),
            [this](vortex_msgs::msg::LandmarkArray::ConstSharedPtr msg) {
                on_detections(msg);
            });
        wipe_sub_ = create_subscription<std_msgs::msg::Empty>(
            get_parameter_or<std::string>("topics.mission_wipe",
                                          "mission/wipe"),
            qos::reliable_profile(1),
            [this](std_msgs::msg::Empty::ConstSharedPtr) { on_wipe(); });

        landmarks_pub_ = create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            "landmark_server/landmarks",
            qos::reliable_transient_local_profile(1));
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "landmark_server/markers",
            qos::reliable_transient_local_profile(1));
        nis_pub_ = create_publisher<std_msgs::msg::Float64>(
            "landmark_server/nis", qos::reliable_profile(10));

        set_premap_srv_ = create_service<vortex_msgs::srv::SetPremap>(
            "landmark_server/set_premap",
            [this](const vortex_msgs::srv::SetPremap::Request::SharedPtr req,
                   vortex_msgs::srv::SetPremap::Response::SharedPtr res) {
                on_set_premap(*req, *res);
            });
        get_premap_srv_ = create_service<std_srvs::srv::Trigger>(
            "landmark_server/get_premap",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                res->success = true;
                res->message = premap_to_yaml(premap_);
            });

        spdlog::info(
            "landmark_server: {} classes, prior map {} ({} tasks), map frame "
            "'{}'",
            cfg_.classes.size(), premap_file_.empty() ? "-" : premap_file_,
            premap_.objects.size(), map_frame_);
    }

   private:
    template <typename T>
    void read(const std::string& name, T& value) {
        value = get_parameter_or<T>(name, value);
    }

    void load_config() {
        Params& p = cfg_.params;
        read("keyframe_dist_m", p.keyframe_dist_m);
        read("keyframe_time_s", p.keyframe_time_s);
        read("max_messages_per_keyframe", p.max_messages_per_keyframe);
        read("max_range_m", p.max_range_m);
        read("odom.sigma_trans_per_m", p.odom_sigma_trans_per_m);
        read("odom.sigma_yaw_per_m", p.odom_sigma_yaw_per_m);
        read("odom.min_sigma_trans", p.odom_min_sigma_trans);
        read("odom.min_sigma_yaw", p.odom_min_sigma_yaw);
        read("odom.sigma_roll_pitch", p.odom_sigma_roll_pitch);
        read("odom.sigma_z", p.odom_sigma_z);
        read("odom.attitude_sigma", p.attitude_sigma);
        read("odom.depth_sigma", p.depth_sigma);
        read("detection.bearing_sigma", p.bearing_sigma);
        read("detection.range_sigma_a", p.range_sigma_a);
        read("detection.range_sigma_b", p.range_sigma_b);
        read("detection.orientation_yaw_sigma", p.orientation_yaw_sigma);
        read("detection.orientation_roll_pitch_sigma",
             p.orientation_roll_pitch_sigma);
        read("detection.dcs_phi", p.dcs_phi);
        read("detection.max_merged_per_factor", p.max_merged_per_factor);
        read("association.gate_prob", p.gate_prob);
        read("association.ambiguity_d2", p.ambiguity_d2);
        read("new_landmarks.candidate_radius_m", p.candidate_radius_m);
        read("new_landmarks.confirm_hits", p.confirm_hits);
        read("new_landmarks.confirm_window_s", p.confirm_window_s);
        read("upkeep.merge_radius_m", p.merge_radius_m);
        read("gate.panel_classes", p.gate.panel_classes);
        read("gate.min_separation_m", p.gate.min_separation_m);
        read("gate.max_separation_m", p.gate.max_separation_m);
        read("gate.approach_m", p.gate.approach_m);
        read("gate.depth_below_panel_m", p.gate.depth_below_panel_m);

        // classes.<name>.<key>
        std::set<std::string> names;
        for (const auto& full : list_parameters({"classes"}, 3).names) {
            const auto first = full.find('.');
            const auto second = full.find('.', first + 1);
            if (first != std::string::npos && second != std::string::npos) {
                names.insert(full.substr(first + 1, second - first - 1));
            }
        }
        for (const auto& name : names) {
            const std::string k = "classes." + name + ".";
            ClassConfig c;
            c.name = name;
            int64_t type = -1;
            int64_t subtype = -1;
            read(k + "type", type);
            read(k + "subtype", subtype);
            if (type < 0 || subtype < 0) {
                throw std::runtime_error(fmt::format(
                    "class '{}': type and subtype are required", name));
            }
            c.type = static_cast<std::uint16_t>(type);
            c.subtype = static_cast<std::uint16_t>(subtype);
            read(k + "symmetry_deg", c.symmetry_deg);
            read(k + "has_orientation", c.has_orientation);
            read(k + "prior", c.prior);
            read(k + "prior_radius_m", c.prior_radius_m);
            int64_t max_instances = 0;
            read(k + "max_instances", max_instances);
            c.max_instances = static_cast<int>(max_instances);
            cfg_.classes.push_back(c);
        }
        cfg_.validate();
    }

    void load_premap_file() {
        if (premap_file_.empty()) {
            return;
        }
        try {
            std::vector<std::string> skipped;
            premap_ = load_premap(premap_file_, skipped);
            for (const auto& label : skipped) {
                spdlog::warn("landmark_server: prior map entry '{}' skipped",
                             label);
            }
        } catch (const std::exception& e) {
            spdlog::warn("landmark_server: no prior map: {}", e.what());
        }
        warn_unknown_prior_labels();
    }

    void warn_unknown_prior_labels() const {
        for (const ClassConfig& c : cfg_.classes) {
            if (!c.prior.empty() && !premap_.objects.empty() &&
                premap_.objects.count(c.prior) == 0) {
                spdlog::warn(
                    "landmark_server: class '{}' belongs to task '{}', which "
                    "is not in the prior map: no prior gate for it",
                    c.name, c.prior);
            }
        }
    }

    void on_set_premap(const vortex_msgs::srv::SetPremap::Request& req,
                       vortex_msgs::srv::SetPremap::Response& res) {
        // Poses in reference_frame -> map.
        gtsam::Pose3 T_map_ref;
        if (!is_map_reference(req.reference_frame)) {
            const std::string ref = req.reference_frame == "odom"
                                        ? odom_frame_
                                        : req.reference_frame;
            if (odom_ && ref == odom_frame_) {
                T_map_ref = graph_.map_to_odom();
            } else {
                try {
                    T_map_ref =
                        to_pose3(tf_buffer_
                                     ->lookupTransform(map_frame_, ref,
                                                       tf2::TimePointZero)
                                     .transform);
                } catch (const tf2::TransformException& e) {
                    res.success = false;
                    res.message = fmt::format("no TF {} -> {}: {}", ref,
                                              map_frame_, e.what());
                    return;
                }
            }
        }
        Premap next;
        next.created_at = now_iso8601();
        YAML::Node gui_objects;
        std::string gui_reference;
        for (const auto& o : req.objects) {
            const std::string& label = o.label;
            if (label.rfind(kGuiPrefix, 0) == 0) {
                // The GUI's own drawing: kept as it is.
                const std::string key =
                    label.substr(std::string(kGuiPrefix).size());
                if (key.rfind("reference_frame/", 0) == 0) {
                    gui_reference = key.substr(16);
                } else if (key.rfind("object/", 0) == 0) {
                    // Kept as the GUI sent it, 7 significant digits.
                    const auto& p = o.pose;
                    const auto list = [](std::initializer_list<double> v) {
                        YAML::Node seq(YAML::NodeType::Sequence);
                        seq.SetStyle(YAML::EmitterStyle::Flow);
                        for (const double x : v) {
                            seq.push_back(fmt::format(
                                "{:.7g}", std::abs(x) < 1e-9 ? 0.0 : x));
                        }
                        return seq;
                    };
                    YAML::Node n;
                    n["position"] =
                        list({p.position.x, p.position.y, p.position.z});
                    n["orientation"] = list({p.orientation.x, p.orientation.y,
                                             p.orientation.z, p.orientation.w});
                    gui_objects[key.substr(7)] = n;
                }
                continue;
            }
            next.objects[label] = T_map_ref * to_pose3(o.pose);
        }
        if (next.objects.empty()) {
            res.success = false;
            res.message = "no objects in the request; prior map unchanged";
            return;
        }
        if (gui_objects) {
            next.gui_state["reference_frame"] = gui_reference;
            next.gui_state["objects"] = gui_objects;
        }
        std::string backup;
        if (!premap_file_.empty()) {
            try {
                backup = save_premap(premap_file_, next);
            } catch (const std::exception& e) {
                res.success = false;
                res.message = fmt::format("not saved: {}", e.what());
                return;
            }
        }
        premap_ = next;
        warn_unknown_prior_labels();
        res.success = true;
        res.message = fmt::format(
            "prior map set with {} tasks, saved to {}{}",
            premap_.objects.size(),
            premap_file_.empty() ? "nowhere (no premap_file)" : premap_file_,
            backup.empty() ? "" : " (old one: " + backup + ")");
        spdlog::info("landmark_server: {}", res.message);
        if (odom_) {
            publish_map(odom_->t);
        }
    }

    void reset(const gtsam::Pose3& T_odom_base, double t) {
        graph_.reset(cfg_.params, T_odom_base, t);
        candidates_.clear();
        pending_.clear();
        nis_.clear();
        next_id_ = 1;
        prev_keyframe_t_ = t;
        publish_map(t);
    }

    void on_wipe() {
        if (!odom_) {
            return;  // the map starts with the first odometry
        }
        reset(odom_->T, odom_->t);
        spdlog::info("landmark_server: new map at the vehicle (mission/wipe)");
    }

    void on_odom(const nav_msgs::msg::Odometry& msg) {
        const double t = rclcpp::Time(msg.header.stamp).seconds();
        const gtsam::Pose3 T = to_pose3(msg.pose.pose);
        odom_frame_ = msg.header.frame_id;
        const bool first = !odom_;
        odom_ = OdomSample{T, t};
        if (first) {
            reset(T, t);
        } else {
            const int kf = graph_.last_keyframe();
            const double dist =
                (graph_.keyframe_odom(kf).translation() - T.translation())
                    .norm();
            if (dist >= cfg_.params.keyframe_dist_m ||
                t - graph_.keyframe_time(kf) >= cfg_.params.keyframe_time_s) {
                process_keyframe(T, t);
            }
        }
        publish_tf(t);
    }

    void on_detections(
        const vortex_msgs::msg::LandmarkArray::ConstSharedPtr& msg) {
        if (msg->landmarks.empty()) {
            return;
        }
        // Messages wait for the next keyframe, per source (frame and the
        // types it carries): the newest max_messages_per_keyframe.
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
        while (q.size() > static_cast<std::size_t>(
                              cfg_.params.max_messages_per_keyframe)) {
            q.pop_front();
        }
    }

    void process_keyframe(const gtsam::Pose3& T_odom_base, double t) {
        const Params& p = cfg_.params;
        const int kf = graph_.add_keyframe(T_odom_base, t);
        graph_.update();
        const gtsam::Pose3 T_map_base = graph_.keyframe_pose(kf);
        const auto priors = premap_.positions();

        // Each message is associated on its own; the detections matched to
        // one landmark become one factor (their mean, Measurement::merged).
        std::map<int, std::pair<Measurement, gtsam::Point3>> matched;
        double t_obs = 0.0;
        for (const auto& [source, msgs] : pending_) {
            for (const auto& msg : msgs) {
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
                candidates_.add(detections, a.unmatched, kf, T_map_base, t_det,
                                p, priors);
            }
        }
        pending_.clear();
        for (const auto& [id, agg] : matched) {
            graph_.add_observation(kf, id, agg.first, t_obs);
        }
        if (!matched.empty()) {
            graph_.update();
        }

        for (const Candidate& c : candidates_.take_confirmed(graph_, t)) {
            const int id = next_id_++;
            std::optional<double> yaw;
            for (const Hit& h : c.hits) {
                yaw = h.yaw ? h.yaw : yaw;
            }
            graph_.add_landmark(
                id, *c.cls,
                gtsam::Pose3(gtsam::Rot3::Yaw(yaw.value_or(0.0)), c.position),
                c.hits.front().t);
            for (const Hit& h : c.hits) {
                graph_.add_observation(h.kf, id, h.z, h.t);
            }
            graph_.update();
            spdlog::info(
                "landmark_server: new landmark {} ({}) at [{:.2f}, {:.2f}, "
                "{:.2f}] from {} detections",
                id, c.cls->name, c.position.x(), c.position.y(), c.position.z(),
                c.hits.size());
        }

        warn_prior_rejects(t);
        // Two landmarks of a class this close whose positions agree are one
        // object (e.g. a copy made while the odometry had drifted).
        for (const auto& [keep, drop] : graph_.retire_duplicates(
                 p.merge_radius_m, chi2_threshold(p.gate_prob, 3))) {
            spdlog::info("landmark_server: landmark {} is a duplicate of {}",
                         drop, keep);
        }
        publish_map(t);
        prev_keyframe_t_ = t;
    }

    /// Many detections of a class far from its prior: the prior map is
    /// probably wrong (no landmark of the task can be made). Every 10 s.
    void warn_prior_rejects(double t) {
        for (const auto& [cls, n] : candidates_.take_prior_rejects()) {
            auto& [count, nearest] =
                prior_rejects_.try_emplace(cls, 0, n.second).first->second;
            count += n.first;
            nearest = std::min(nearest, n.second);
        }
        if (t - last_prior_warn_ < kPriorWarnPeriodS) {
            return;
        }
        for (const auto& [cls, n] : prior_rejects_) {
            if (n.first >= kPriorWarnMinCount) {
                const ClassConfig* c = cfg_.find_class(cls);
                spdlog::warn(
                    "landmark_server: {} detections of {} rejected by the "
                    "prior map in {:.0f} s, the nearest {:.1f} m from task "
                    "'{}' (prior_radius_m {:.1f}): is the prior map right?",
                    n.first, cls, kPriorWarnPeriodS, n.second, c->prior,
                    c->prior_radius_m);
            }
        }
        prior_rejects_.clear();
        last_prior_warn_ = t;
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
                spdlog::warn("landmark_server: no TF {} -> {}: {}",
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
                        "landmark_server: type {} subtype {} is not a "
                        "configured class, ignored",
                        l.type.value, l.subtype.value);
                }
                continue;
            }
            const gtsam::Pose3 T_base_obj =
                T_base_frame * to_pose3(l.pose.pose);
            const double range = T_base_obj.translation().norm();
            if (range < kMinRangeM || range > cfg_.params.max_range_m) {
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

    void publish_tf(double t) {
        geometry_msgs::msg::TransformStamped tf =
            frame(odom_frame_, graph_.map_to_odom());
        tf.child_frame_id = odom_frame_;
        tf.header.stamp = to_stamp(t);
        tf_broadcaster_->sendTransform(tf);

        // The map's frames with the same stamp: a lookup from odom gets them
        // where the drifted vehicle needs them.
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

    /// The landmarks shown: per class at most max_instances, the most
    /// observed (the rest keep matching their detections, unseen).
    std::vector<LandmarkState> shown_landmarks() const {
        std::vector<LandmarkState> all = graph_.landmarks();
        std::stable_sort(all.begin(), all.end(),
                         [](const LandmarkState& a, const LandmarkState& b) {
                             return a.n_obs > b.n_obs;
                         });
        std::map<std::string, int> count;
        std::vector<LandmarkState> out;
        for (const LandmarkState& l : all) {
            const int n = ++count[l.cls.name];
            if (l.cls.max_instances == 0 || n <= l.cls.max_instances) {
                out.push_back(l);
            }
        }
        return out;
    }

    void publish_map(double t) {
        graph_.update_covariances();
        const std::vector<LandmarkState> shown = shown_landmarks();

        frames_.clear();
        // Where the run started (return home).
        frames_.push_back(frame("start", graph_.keyframe_pose(0)));
        // Where each task should be: the search point before it is seen.
        for (const auto& [label, pose] : premap_.objects) {
            frames_.push_back(frame("prior_" + label, pose));
        }
        // Per class the landmark seen most often: a real object is seen far
        // more often than a phantom or a confused detection.
        std::set<std::string> classes;
        for (const LandmarkState& l : shown) {
            classes.insert(l.cls.name);
        }
        for (const auto& cls : classes) {
            frames_.push_back(frame(cls, best_of(shown, cls)->pose));
        }
        for (const NamedPose& g :
             gate_frames(shown, cfg_.params.gate,
                         graph_.keyframe_pose(0).translation())) {
            frames_.push_back(frame(g.name, g.pose));
        }

        vortex_msgs::msg::LandmarkTrackArray array;
        array.header.stamp = to_stamp(t);
        array.header.frame_id = map_frame_;
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);
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
            track.confirmed = true;
            track.retained = l.last_seen < prev_keyframe_t_;
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
        m.color.r = track.retained ? 0.6 : 0.1;
        m.color.g = track.retained ? 0.6 : 0.8;
        m.color.b = track.retained ? 0.6 : 0.2;
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

    Config cfg_;
    Premap premap_;
    std::string premap_file_;
    LandmarkGraph graph_;
    Candidates candidates_;
    std::map<std::string,
             std::deque<vortex_msgs::msg::LandmarkArray::ConstSharedPtr>>
        pending_;
    std::optional<OdomSample> odom_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string frame_prefix_;
    std::vector<geometry_msgs::msg::TransformStamped> frames_;
    double last_frames_t_{0.0};
    double prev_keyframe_t_{0.0};
    std::map<std::string, std::pair<int, double>> prior_rejects_;
    double last_prior_warn_{0.0};
    int next_id_{1};
    std::deque<double> nis_;
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
    rclcpp::Service<vortex_msgs::srv::SetPremap>::SharedPtr set_premap_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_premap_srv_;
};

}  // namespace vortex::landmark_server

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::landmark_server::LandmarkServerNode)
