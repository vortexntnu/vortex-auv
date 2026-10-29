#include <spdlog/spdlog.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vortex/utils/ros/landmark_names.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex_msgs/action/measure_landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_track_array.hpp>
#include <vortex_msgs/srv/reset_landmarks.hpp>
#include <vortex_msgs/srv/set_premap.hpp>

#include "landmark_server/config.hpp"
#include "landmark_server/detection_check.hpp"
#include "landmark_server/geometry.hpp"
#include "landmark_server/map.hpp"
#include "landmark_server/measure.hpp"
#include "landmark_server/premap.hpp"
#include "landmark_server/targets.hpp"

namespace vortex::landmark_server {

namespace {

constexpr double kMinRangeM = 0.05;
constexpr double kFramePeriodS = 0.1;
constexpr auto kMapPeriod = std::chrono::milliseconds(500);
constexpr double kTfWaitS = 1.0;
constexpr double kRetainedAfterS = 1.0;
constexpr double kLiveS = 1.0;  // <class>/live lives this long after a hit
constexpr double kNoOrientationVariance = 1000.0;
constexpr const char* kGuiPrefix = "__gui/";
constexpr double kPriorWarnPeriodS = 10.0;
constexpr int kPriorWarnMinCount = 20;

Pose to_pose(const geometry_msgs::msg::Pose& p) {
    Pose T = Pose::Identity();
    T.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                    p.orientation.y, p.orientation.z)
                     .normalized()
                     .toRotationMatrix();
    T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    return T;
}

Pose to_pose(const geometry_msgs::msg::Transform& t) {
    Pose T = Pose::Identity();
    T.linear() = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y,
                                    t.rotation.z)
                     .normalized()
                     .toRotationMatrix();
    T.translation() =
        Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
    return T;
}

geometry_msgs::msg::Pose to_msg(const Pose& T) {
    geometry_msgs::msg::Pose p;
    p.position.x = T.translation().x();
    p.position.y = T.translation().y();
    p.position.z = T.translation().z();
    const Eigen::Quaterniond q(T.rotation());
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    return p;
}

builtin_interfaces::msg::Time to_stamp(double t) {
    return rclcpp::Time(static_cast<int64_t>(t * 1e9));
}

/// ROS covariance order: x, y, z, then rotation. Roll and pitch unknown.
std::array<double, 36> to_ros_cov(const Landmark& l) {
    std::array<double, 36> a{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            a[r * 6 + c] = l.cov(r, c);
        }
    }
    a[21] = a[28] = kNoOrientationVariance;
    a[35] = l.yaw_known ? l.yaw_var : kNoOrientationVariance;
    return a;
}

constexpr const char* kSlalomRow = "slalom_row";

using MeasureLandmark = vortex_msgs::action::MeasureLandmark;
using MeasureHandle = rclcpp_action::ServerGoalHandle<MeasureLandmark>;

Pose landmark_pose(const Landmark& l) {
    return yaw_pose(l.yaw_known ? l.yaw : 0.0, l.position);
}

}  // namespace

class LandmarkServerNode : public rclcpp::Node {
   public:
    explicit LandmarkServerNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node(
              "landmark_server_node",
              rclcpp::NodeOptions(options)
                  .automatically_declare_parameters_from_overrides(true)) {
        load_config();
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
        heading_sub_ = create_subscription<std_msgs::msg::Empty>(
            get_parameter_or<std::string>("topics.mission_set_heading",
                                          "mission/set_heading"),
            qos::reliable_profile(1),
            [this](std_msgs::msg::Empty::ConstSharedPtr) { on_set_heading(); });

        landmarks_pub_ = create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            "landmark_server/landmarks",
            qos::reliable_transient_local_profile(1));
        debug_ = get_parameter_or<bool>("debug", false);
        if (debug_) {
            markers_pub_ =
                create_publisher<visualization_msgs::msg::MarkerArray>(
                    "landmark_server/markers",
                    qos::reliable_transient_local_profile(1));
            raw_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                "landmark_server/raw_detections", qos::reliable_profile(10));
        }

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
                res->message = premap_to_yaml(premap_) + prior_radius_yaml();
            });

        map_timer_ = create_wall_timer(kMapPeriod, [this] { update_map(); });

        measure_server_ = rclcpp_action::create_server<MeasureLandmark>(
            this, "landmark_server/measure",
            [this](const rclcpp_action::GoalUUID&,
                   std::shared_ptr<const MeasureLandmark::Goal> goal) {
                return accept_measure(*goal);
            },
            [](const std::shared_ptr<MeasureHandle>&) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<MeasureHandle>& handle) {
                start_measure(handle);
            });
        reset_srv_ = create_service<vortex_msgs::srv::ResetLandmarks>(
            "landmark_server/reset",
            [this](
                const vortex_msgs::srv::ResetLandmarks::Request::SharedPtr req,
                vortex_msgs::srv::ResetLandmarks::Response::SharedPtr res) {
                on_reset(*req, *res);
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
        read("max_range_m", p.max_range_m);
        read("map.cluster_radius_m", p.cluster_radius_m);
        read("map.confirm_hits", p.confirm_hits);
        read("map.max_hits_per_class", p.max_hits_per_class);
        read("map.max_hit_age_s", p.max_hit_age_s);
        read("measure.duration_s", p.measure_duration_s);
        read("measure.max_range_m", p.measure_max_range_m);
        read("measure.min_hits", p.measure_min_hits);
        read("measure.max_spread_m", p.measure_max_spread_m);
        read("gate.panel_classes", p.gate.panel_classes);
        read("gate.min_separation_m", p.gate.min_separation_m);
        read("gate.max_separation_m", p.gate.max_separation_m);
        read("gate.approach_m", p.gate.approach_m);
        read("gate.depth_below_panel_m", p.gate.depth_below_panel_m);

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
            std::string type;
            std::string subtype;
            read(k + "type", type);
            read(k + "subtype", subtype);
            const auto type_value =
                vortex::utils::landmarks::landmark_type_from_string(type);
            const auto subtype_value =
                vortex::utils::landmarks::landmark_subtype_from_string(subtype);
            if (!type_value || !subtype_value) {
                throw std::runtime_error(fmt::format(
                    "class '{}': type '{}' / subtype '{}': give the names of "
                    "the vortex_msgs LandmarkType and LandmarkSubtype "
                    "constants",
                    name, type, subtype));
            }
            c.type = *type_value;
            c.subtype = *subtype_value;
            read(k + "symmetry_deg", c.symmetry_deg);
            read(k + "has_orientation", c.has_orientation);
            read(k + "prior", c.prior);
            read(k + "prior_radius_m", c.prior_radius_m);
            read(k + "group", c.group);
            if (c.group.empty()) {
                c.group = c.name;
            }
            int64_t max_instances = 0;
            read(k + "max_instances", max_instances);
            c.max_instances = static_cast<int>(max_instances);
            const std::string b = "markers.boxes." + name + ".";
            if (std::vector<double> size;
                read(b + "size", size), size.size() == 3) {
                MarkerBox box;
                std::copy_n(size.begin(), 3, box.size.begin());
                std::vector<double> v;
                if (read(b + "offset", v); v.size() == 3) {
                    std::copy_n(v.begin(), 3, box.offset.begin());
                }
                v.clear();
                if (read(b + "color", v); v.size() == 3) {
                    box.color = {v[0], v[1], v[2]};
                }
                read(b + "solid", box.solid);
                c.box = box;
            }
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

    /// Prior radius per task, for the GUI.
    std::string prior_radius_yaml() const {
        std::map<std::string, double> radius;
        for (const ClassConfig& c : cfg_.classes) {
            if (!c.prior.empty()) {
                radius[c.prior] = std::max(radius[c.prior], c.prior_radius_m);
            }
        }
        for (const auto& [task, r] : premap_.radius) {
            radius[task] = r;
        }
        std::string out = "prior_radius_m:\n";
        for (const auto& [task, r] : radius) {
            out += fmt::format("  {}: {:.7g}\n", task, r);
        }
        return radius.empty() ? "" : out;
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
        Pose T_map_ref = Pose::Identity();
        if (!is_map_reference(req.reference_frame)) {
            const std::string ref = req.reference_frame == "odom"
                                        ? odom_frame_
                                        : req.reference_frame;
            if (odom_ && ref == odom_frame_) {
                T_map_ref = T_map_odom_;
            } else {
                try {
                    T_map_ref =
                        to_pose(tf_buffer_
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
                const std::string key =
                    label.substr(std::string(kGuiPrefix).size());
                if (key.rfind("reference_frame/", 0) == 0) {
                    gui_reference = key.substr(16);
                } else if (key.rfind("object/", 0) == 0) {
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
            next.objects[label] = T_map_ref * to_pose(o.pose);
            if (o.radius > 0.0) {
                next.radius[label] = o.radius;
            }
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
            publish_map();
        }
    }

    /// The map frame starts at the vehicle, levelled. Its X is the course
    /// heading if one is set, otherwise the way the vehicle points now. It
    /// stays fixed until the next wipe.
    void reset(const Pose& T_odom_base) {
        const double vehicle_yaw = yaw_of(T_odom_base.rotation());
        const double start_yaw =
            course_yaw_ ? wrap(vehicle_yaw - *course_yaw_) : 0.0;
        const Eigen::Matrix3d R = T_odom_base.rotation();
        // Keep the vehicle's roll and pitch in the start pose, as before.
        const Eigen::Matrix3d level =
            Eigen::AngleAxisd(-vehicle_yaw, Eigen::Vector3d::UnitZ()).matrix() *
            R;
        T_map_start_ = Pose::Identity();
        T_map_start_.linear() =
            Eigen::AngleAxisd(start_yaw, Eigen::Vector3d::UnitZ()).matrix() *
            level;
        T_map_start_.translation() =
            Eigen::Vector3d(0.0, 0.0, T_odom_base.translation().z());
        T_map_odom_ = T_map_start_ * T_odom_base.inverse();
        map_.clear();
        last_hit_.clear();
        pending_.clear();
        publish_map();
    }

    void on_wipe() {
        if (!odom_) {
            return;
        }
        reset(odom_->T);
        if (course_yaw_) {
            spdlog::info(
                "landmark_server: new map at the vehicle, which is turned "
                "{:.0f} deg from the course heading (mission/wipe)",
                yaw_of(T_map_start_.rotation()) * 180.0 / M_PI);
        } else {
            spdlog::info(
                "landmark_server: new map at the vehicle, facing the way it "
                "points now (mission/wipe)");
        }
    }

    void on_set_heading() {
        if (!odom_) {
            return;
        }
        course_yaw_ = yaw_of(odom_->T.rotation());
        reset(odom_->T);
        spdlog::info(
            "landmark_server: course heading set to where the vehicle points "
            "now, new map at the vehicle (mission/set_heading)");
    }

    void on_odom(const nav_msgs::msg::Odometry& msg) {
        const double t = rclcpp::Time(msg.header.stamp).seconds();
        odom_frame_ = msg.header.frame_id;
        const bool first = !odom_;
        odom_ = OdomSample{to_pose(msg.pose.pose), t};
        if (first) {
            reset(odom_->T);
        }
        publish_tf(t);
    }

    void on_detections(
        const vortex_msgs::msg::LandmarkArray::ConstSharedPtr& msg) {
        if (msg->landmarks.empty()) {
            return;
        }
        if (debug_) {
            publish_raw_detections(*msg);
        }
        pending_.push_back(msg);
    }

    /// Every detection as it came in, a small sphere that lives 0.3 s.
    void publish_raw_detections(const vortex_msgs::msg::LandmarkArray& msg) {
        visualization_msgs::msg::MarkerArray out;
        for (const auto& l : msg.landmarks) {
            visualization_msgs::msg::Marker m;
            m.header = msg.header;
            m.ns = msg.header.frame_id;  // one set of ids per source
            m.id = static_cast<int>(out.markers.size());
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.pose = l.pose.pose;
            m.scale.x = m.scale.y = m.scale.z = 0.12;
            m.color = type_color(l.type.value);
            m.lifetime = rclcpp::Duration::from_seconds(0.3);
            out.markers.push_back(m);
        }
        raw_pub_->publish(out);
    }

    void update_map() {
        if (!odom_) {
            return;
        }
        // A message whose TF is not there yet is tried again for a while.
        std::vector<vortex_msgs::msg::LandmarkArray::ConstSharedPtr> wait;
        for (const auto& msg : pending_) {
            if (!add_hits(*msg) &&
                odom_->t - rclcpp::Time(msg->header.stamp).seconds() <
                    kTfWaitS) {
                wait.push_back(msg);
            }
        }
        pending_ = std::move(wait);
        tick_measure();
        const Params& p = cfg_.params;
        map_.update(p.cluster_radius_m, p.confirm_hits, p.max_hit_age_s,
                    odom_->t);
        warn_prior_rejects(odom_->t);
        publish_map();
    }

    /// Warns when the prior map rejects many detections of a class.
    void warn_prior_rejects(double t) {
        if (t - last_prior_warn_ < kPriorWarnPeriodS) {
            return;
        }
        for (const auto& [cls, n] : prior_rejects_) {
            if (n.first >= kPriorWarnMinCount) {
                const ClassConfig* c = cfg_.find_class(cls);
                spdlog::warn(
                    "landmark_server: {} detections of {} rejected by the "
                    "prior map in {:.0f} s, the nearest {:.1f} m from task "
                    "'{}': is the prior map right?",
                    n.first, cls, kPriorWarnPeriodS, n.second, c->prior);
            }
        }
        prior_rejects_.clear();
        last_prior_warn_ = t;
    }

    /// Prior radius of a task: the GUI's, else the largest of its classes.
    double prior_radius(const ClassConfig& cls) const {
        const auto r = premap_.radius.find(cls.prior);
        return r != premap_.radius.end() ? r->second : cls.prior_radius_m;
    }

    /// Stores the detections of a message as hits in odom. False if its TF
    /// is missing.
    bool add_hits(const vortex_msgs::msg::LandmarkArray& msg) {
        const std::string& frame_id = msg.header.frame_id;
        if ((frame_id == odom_frame_ || frame_id == map_frame_) &&
            world_frame_sources_.insert(frame_id).second) {
            spdlog::warn(
                "landmark_server: detections in '{}' are used, but detectors "
                "should publish in the camera frame with the image stamp",
                frame_id);
        }
        Pose T_odom_frame = Pose::Identity();
        if (frame_id == map_frame_) {
            T_odom_frame = T_map_odom_.inverse();
        } else if (frame_id != odom_frame_) {
            try {
                T_odom_frame =
                    to_pose(tf_buffer_
                                ->lookupTransform(odom_frame_, frame_id,
                                                  msg.header.stamp)
                                .transform);
            } catch (const tf2::TransformException& e) {
                spdlog::debug("landmark_server: no TF {} -> {} yet: {}",
                              frame_id, odom_frame_, e.what());
                return false;
            }
        }
        const double t = rclcpp::Time(msg.header.stamp).seconds();
        const Eigen::Vector3d vehicle = odom_->T.translation();
        const Pose T_odom_map = T_map_odom_.inverse();
        const auto priors = premap_.positions();

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
            const auto checked = check_detection(l.pose);
            if (!checked) {
                if (bad_.insert({l.type.value, l.subtype.value}).second) {
                    spdlog::warn(
                        "landmark_server: type {} subtype {} sent a NaN "
                        "position or a non-unit quaternion, dropped",
                        l.type.value, l.subtype.value);
                }
                continue;
            }
            Hit h;
            h.cls = cls;
            h.t = t;
            h.position = T_odom_frame * checked->position;
            if (checked->rotation) {
                h.yaw = yaw_of(T_odom_frame.rotation() *
                               checked->rotation->toRotationMatrix());
            }
            const double range = (h.position - vehicle).norm();
            if (measuring_ && t >= measuring_->start_t &&
                range <= measuring_->max_range_m &&
                (measuring_->cls == cls->name ||
                 (measuring_->cls == kSlalomRow &&
                  cls->type == vortex_msgs::msg::LandmarkType::SLALOM_PIPE))) {
                measuring_->hits.push_back(h);
            }
            if (range < kMinRangeM || range > cfg_.params.max_range_m) {
                continue;
            }
            if (const auto prior = priors.find(cls->prior);
                prior != priors.end()) {
                const double d =
                    (h.position - T_odom_map * prior->second).head<2>().norm();
                if (d > prior_radius(*cls)) {
                    auto& [count, nearest] =
                        prior_rejects_.try_emplace(cls->name, 0, d)
                            .first->second;
                    count++;
                    nearest = std::min(nearest, d);
                    continue;
                }
            }
            if (t >= last_hit_[cls->name].t) {
                last_hit_[cls->name] = h;
            }
            map_.add(h,
                     static_cast<std::size_t>(cfg_.params.max_hits_per_class));
        }
        return true;
    }

    rclcpp_action::GoalResponse accept_measure(
        const MeasureLandmark::Goal& goal) {
        if (measuring_ || !odom_ ||
            (goal.class_name != kSlalomRow &&
             !cfg_.find_class(goal.class_name))) {
            spdlog::warn(
                "landmark_server: measure '{}' rejected: {}", goal.class_name,
                measuring_ ? "another measurement is running"
                           : (!odom_ ? "no odometry yet" : "no such class"));
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void start_measure(const std::shared_ptr<MeasureHandle>& handle) {
        const auto& g = *handle->get_goal();
        const Params& p = cfg_.params;
        Measuring m;
        m.handle = handle;
        m.cls = g.class_name;
        m.out_frame =
            frame_prefix_ +
            (g.out_frame.empty() ? g.class_name + "/measured" : g.out_frame);
        m.fallback_from = g.fallback_from;
        m.start_t = odom_->t;
        m.end_t = odom_->t +
                  (g.duration_s > 0.0 ? g.duration_s : p.measure_duration_s);
        m.max_range_m =
            g.max_range_m > 0.0 ? g.max_range_m : p.measure_max_range_m;
        m.settings.cluster_radius_m = p.cluster_radius_m;
        m.settings.min_hits = g.min_hits > 0 ? g.min_hits : p.measure_min_hits;
        m.settings.max_spread_m =
            g.max_spread_m > 0.0 ? g.max_spread_m : p.measure_max_spread_m;
        measuring_ = std::move(m);
        spdlog::info("landmark_server: measuring {} for {:.1f} s", g.class_name,
                     measuring_->end_t - measuring_->start_t);
    }

    void tick_measure() {
        if (!measuring_) {
            return;
        }
        Measuring& m = *measuring_;
        if (m.handle->is_canceling()) {
            auto result = std::make_shared<MeasureLandmark::Result>();
            result->hits = static_cast<int>(m.hits.size());
            result->message = "canceled";
            m.handle->canceled(result);
            measuring_.reset();
            return;
        }
        if (odom_->t < m.end_t) {
            auto fb = std::make_shared<MeasureLandmark::Feedback>();
            fb->hits = static_cast<int>(m.hits.size());
            m.handle->publish_feedback(fb);
            return;
        }
        const Measurement r =
            m.cls == kSlalomRow
                ? measure_slalom_row(m.hits, odom_->T, m.settings)
                : measure_object(m.hits, odom_->T, m.settings);
        auto result = std::make_shared<MeasureLandmark::Result>();
        result->success = r.ok;
        result->hits = r.hits;
        result->spread_m = r.spread_m;
        result->message = r.message;
        if (r.ok) {
            measured_[m.out_frame] = r.pose;
        } else if (!m.fallback_from.empty()) {
            try {
                measured_[m.out_frame] = to_pose(
                    tf_buffer_
                        ->lookupTransform(odom_frame_,
                                          frame_prefix_ + m.fallback_from,
                                          tf2::TimePointZero)
                        .transform);
                result->used_fallback = true;
                result->message += ", copied " + m.fallback_from;
            } catch (const tf2::TransformException& e) {
                result->message += ", no fallback: " + std::string(e.what());
            }
        }
        spdlog::info("landmark_server: measure {} -> {}: {}{}", m.cls,
                     m.out_frame, r.ok ? "ok, " : "failed, ", result->message);
        if (r.ok) {
            m.handle->succeed(result);
        } else {
            m.handle->abort(result);
        }
        measuring_.reset();
    }

    void on_reset(const vortex_msgs::srv::ResetLandmarks::Request& req,
                  vortex_msgs::srv::ResetLandmarks::Response& res) {
        if (req.class_name.empty()) {
            map_.clear();
            last_hit_.clear();
            res.message = "all classes reset";
        } else if (cfg_.find_class(req.class_name)) {
            map_.clear_class(req.class_name);
            last_hit_.erase(req.class_name);
            res.message = req.class_name + " reset";
        } else {
            res.message = "no class '" + req.class_name + "'";
            return;
        }
        res.success = true;
        spdlog::info("landmark_server: {}", res.message);
        publish_map();
    }

    /// targets.<name>: <parent>/<name> for every parent in TF, offset in the
    /// parent's axes. Read every time, so ros2 param set works.
    void add_targets() {
        std::set<std::string> names;
        for (const auto& full : list_parameters({"targets"}, 3).names) {
            const auto first = full.find('.');
            const auto second = full.find('.', first + 1);
            if (first != std::string::npos && second != std::string::npos) {
                names.insert(full.substr(first + 1, second - first - 1));
            }
        }
        constexpr double kDeg = M_PI / 180.0;
        for (const auto& name : names) {
            const std::string k = "targets." + name + ".";
            std::vector<std::string> parents;
            read(k + "parents", parents);
            double v[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            const char* keys[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
            for (int i = 0; i < 6; ++i) {
                const auto param = get_parameter_or<rclcpp::Parameter>(
                    k + keys[i], rclcpp::Parameter());
                if (param.get_type() ==
                    rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    v[i] = param.as_double();
                } else if (param.get_type() ==
                           rclcpp::ParameterType::PARAMETER_INTEGER) {
                    v[i] = static_cast<double>(param.as_int());
                }
            }
            Pose offset = Pose::Identity();
            offset.linear() =
                (Eigen::AngleAxisd(v[5] * kDeg, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(v[4] * kDeg, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(v[3] * kDeg, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
            offset.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
            for (const auto& parent : parents) {
                const std::string pf = frame_prefix_ + parent;
                const bool ours = std::any_of(
                    frames_.begin(), frames_.end(),
                    [&](const auto& f) { return f.child_frame_id == pf; });
                if (ours || tf_buffer_->_frameExists(pf)) {
                    frames_.push_back(frame(pf, pf + "/" + name, offset));
                }
            }
        }
    }

    void publish_tf(double t) {
        geometry_msgs::msg::TransformStamped tf =
            frame(map_frame_, odom_frame_, T_map_odom_);
        tf.header.stamp = to_stamp(t);
        tf_broadcaster_->sendTransform(tf);

        // Same stamp as map->odom so lookups from odom are consistent.
        if (t - last_frames_t_ >= kFramePeriodS || t < last_frames_t_) {
            for (auto& f : frames_) {
                f.header.stamp = tf.header.stamp;
            }
            tf_broadcaster_->sendTransform(frames_);
            last_frames_t_ = t;
        }
    }

    geometry_msgs::msg::TransformStamped frame(const std::string& parent,
                                               const std::string& child,
                                               const Pose& pose) const {
        geometry_msgs::msg::TransformStamped f;
        f.header.frame_id = parent;
        f.child_frame_id = child;
        const auto p = to_msg(pose);
        f.transform.translation.x = p.position.x;
        f.transform.translation.y = p.position.y;
        f.transform.translation.z = p.position.z;
        f.transform.rotation = p.orientation;
        return f;
    }

    /// At most max_instances per class, the most observed first.
    std::vector<Landmark> shown_landmarks() const {
        std::vector<Landmark> all = map_.landmarks();
        std::stable_sort(all.begin(), all.end(),
                         [](const Landmark& a, const Landmark& b) {
                             return a.n_obs > b.n_obs;
                         });
        std::map<std::string, int> count;
        std::vector<Landmark> out;
        for (const Landmark& l : all) {
            const int n = ++count[l.cls.name];
            if (l.cls.max_instances == 0 || n <= l.cls.max_instances) {
                out.push_back(l);
            }
        }
        return out;
    }

    void publish_map() {
        if (odom_frame_.empty()) {
            return;
        }
        const double t = odom_ ? odom_->t : 0.0;
        const std::vector<Landmark> shown = shown_landmarks();
        const std::string odom = odom_frame_;

        frames_.clear();
        frames_.push_back(
            frame(map_frame_, frame_prefix_ + "start", T_map_start_));
        for (const auto& [label, pose] : premap_.objects) {
            frames_.push_back(
                frame(map_frame_, frame_prefix_ + "prior_" + label, pose));
        }
        // The most observed landmark of each class gets the class frame.
        std::set<std::string> classes;
        for (const Landmark& l : shown) {
            classes.insert(l.cls.name);
        }
        for (const auto& cls : classes) {
            frames_.push_back(frame(odom, frame_prefix_ + cls,
                                    landmark_pose(*best_of(shown, cls))));
        }
        // Not seen yet: the class frame is its task in the prior map.
        for (const ClassConfig& c : cfg_.classes) {
            const auto prior = premap_.objects.find(c.prior);
            if (!classes.count(c.name) && prior != premap_.objects.end()) {
                frames_.push_back(
                    frame(map_frame_, frame_prefix_ + c.name, prior->second));
            }
        }
        for (const auto& [name, h] : last_hit_) {
            if (t - h.t <= kLiveS) {
                const Eigen::Vector3d to = h.position - odom_->T.translation();
                frames_.push_back(
                    frame(odom, frame_prefix_ + name + "/live",
                          yaw_pose(h.yaw.value_or(std::atan2(to.y(), to.x())),
                                   h.position)));
            }
        }
        const Eigen::Vector3d start =
            (T_map_odom_.inverse() * T_map_start_).translation();
        for (const NamedPose& g : gate_frames(shown, cfg_.params.gate, start)) {
            frames_.push_back(frame(odom, frame_prefix_ + g.name, g.pose));
        }
        for (const auto& [name, pose] : measured_) {
            frames_.push_back(frame(odom, name, pose));
        }
        add_targets();

        vortex_msgs::msg::LandmarkTrackArray array;
        array.header.stamp = to_stamp(t);
        array.header.frame_id = odom;
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);
        for (const Landmark& l : shown) {
            vortex_msgs::msg::LandmarkTrack track;
            track.header = array.header;
            track.landmark.header = array.header;
            track.landmark.id = l.id;
            track.landmark.type.value = l.cls.type;
            track.landmark.subtype.value = l.cls.subtype;
            track.landmark.pose.pose = to_msg(landmark_pose(l));
            track.landmark.pose.covariance = to_ros_cov(l);
            track.confirmed = true;
            track.retained = t - l.last_seen > kRetainedAfterS;
            track.has_orientation = l.yaw_known;
            track.observations = l.n_obs;
            track.first_seen = to_stamp(l.first_seen);
            track.last_measurement = to_stamp(l.last_seen);
            array.landmark_tracks.push_back(track);
            if (debug_) {
                add_markers(l, track, markers);
            }
        }
        landmarks_pub_->publish(array);
        if (debug_) {
            markers_pub_->publish(markers);
        }
    }

    static std_msgs::msg::ColorRGBA type_color(std::uint16_t type) {
        using LT = vortex_msgs::msg::LandmarkType;
        std::array<float, 3> c{0.7F, 0.7F, 0.7F};
        switch (type) {
            case LT::GATE:
                c = {1.0F, 0.6F, 0.0F};
                break;
            case LT::SLALOM_PIPE:
                c = {0.9F, 0.2F, 0.2F};
                break;
            case LT::TORPEDO_BOARD:
                c = {0.2F, 0.6F, 1.0F};
                break;
            case LT::BIN:
                c = {0.3F, 0.8F, 0.3F};
                break;
            case LT::PATH_MARKER:
                c = {0.9F, 0.9F, 0.2F};
                break;
            case LT::TABLE:
                c = {0.6F, 0.4F, 0.2F};
                break;
            case LT::OCTAGON:
                c = {0.8F, 0.3F, 0.9F};
                break;
            default:
                break;
        }
        std_msgs::msg::ColorRGBA out;
        out.r = c[0];
        out.g = c[1];
        out.b = c[2];
        out.a = 1.0F;
        return out;
    }

    /// Per landmark: a cube or its real-size box, a label, and an arrow
    /// along +X when the yaw is known. Grey when not seen lately.
    void add_markers(const Landmark& l,
                     const vortex_msgs::msg::LandmarkTrack& track,
                     visualization_msgs::msg::MarkerArray& out) const {
        using visualization_msgs::msg::Marker;
        const float alpha = 0.9F;
        const Eigen::Matrix3d R =
            yaw_pose(l.yaw_known ? l.yaw : 0.0, l.position).rotation();
        const Eigen::Quaterniond q(R);

        Marker m;
        m.header = track.header;
        m.id = l.id;
        m.pose.position = track.landmark.pose.pose.position;
        m.pose.orientation.w = 1.0;
        m.color = type_color(l.cls.type);
        if (track.retained) {
            m.color.r = m.color.g = m.color.b = 0.5F;
        }
        m.color.a = alpha;

        if (!l.cls.box || !l.cls.box->solid) {
            Marker point = m;
            point.ns = "landmark";
            point.type = Marker::CUBE;
            point.scale.x = point.scale.y = point.scale.z = 0.25;
            out.markers.push_back(point);
        }

        if (l.cls.box) {
            const MarkerBox& box = *l.cls.box;
            const Eigen::Vector3d centre =
                l.position + R * Eigen::Vector3d(box.offset[0], box.offset[1],
                                                 box.offset[2]);
            Marker b = m;
            b.ns = "structure";
            b.type = Marker::CUBE;
            b.pose.position.x = centre.x();
            b.pose.position.y = centre.y();
            b.pose.position.z = centre.z();
            b.pose.orientation.w = q.w();
            b.pose.orientation.x = q.x();
            b.pose.orientation.y = q.y();
            b.pose.orientation.z = q.z();
            b.scale.x = box.size[0];
            b.scale.y = box.size[1];
            b.scale.z = box.size[2];
            if (box.color && !track.retained) {
                b.color.r = static_cast<float>((*box.color)[0]);
                b.color.g = static_cast<float>((*box.color)[1]);
                b.color.b = static_cast<float>((*box.color)[2]);
            }
            b.color.a = box.solid ? alpha : 0.25F;
            out.markers.push_back(b);
        }

        Marker label = m;
        label.ns = "label";
        label.type = Marker::TEXT_VIEW_FACING;
        label.pose.position.z -= 0.35;
        label.scale.z = 0.18;
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
        label.text = fmt::format("{} #{} (n {}, spread {:.2f} m)", l.cls.name,
                                 l.id, l.n_obs, l.spread_m);
        out.markers.push_back(label);

        if (l.yaw_known) {
            Marker arrow = m;
            arrow.ns = "front";
            arrow.type = Marker::ARROW;
            arrow.pose.orientation.w = q.w();
            arrow.pose.orientation.x = q.x();
            arrow.pose.orientation.y = q.y();
            arrow.pose.orientation.z = q.z();
            arrow.scale.x = 0.8;
            arrow.scale.y = arrow.scale.z = 0.06;
            arrow.color.a = 1.0F;
            out.markers.push_back(arrow);
        }
    }

    struct Measuring {
        std::shared_ptr<MeasureHandle> handle;
        std::string cls;
        std::string out_frame;
        std::string fallback_from;
        double start_t{0.0};
        double end_t{0.0};
        double max_range_m{0.0};
        MeasureSettings settings;
        std::vector<Hit> hits;
    };

    struct OdomSample {
        Pose T{Pose::Identity()};
        double t{0.0};
    };

    Config cfg_;
    Premap premap_;
    std::string premap_file_;
    DetectionMap map_;
    std::optional<Measuring> measuring_;
    std::map<std::string, Hit> last_hit_;
    // Measured frames in odom, kept until measured again.
    std::map<std::string, Pose> measured_;
    std::vector<vortex_msgs::msg::LandmarkArray::ConstSharedPtr> pending_;
    std::optional<OdomSample> odom_;
    // Odometry yaw that points at the course, kept between wipes.
    std::optional<double> course_yaw_;
    Pose T_map_odom_{Pose::Identity()};
    Pose T_map_start_{Pose::Identity()};
    std::string odom_frame_;
    std::string map_frame_;
    std::string frame_prefix_;
    std::vector<geometry_msgs::msg::TransformStamped> frames_;
    double last_frames_t_{0.0};
    std::map<std::string, std::pair<int, double>> prior_rejects_;
    double last_prior_warn_{0.0};
    std::set<std::pair<std::uint16_t, std::uint16_t>> unknown_;
    std::set<std::pair<std::uint16_t, std::uint16_t>> bad_;
    std::set<std::string> world_frame_sources_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heading_sub_;
    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
        detection_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr wipe_sub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        landmarks_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr raw_pub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp_action::Server<MeasureLandmark>::SharedPtr measure_server_;
    rclcpp::Service<vortex_msgs::srv::ResetLandmarks>::SharedPtr reset_srv_;
    bool debug_{false};
    rclcpp::Service<vortex_msgs::srv::SetPremap>::SharedPtr set_premap_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_premap_srv_;
};

}  // namespace vortex::landmark_server

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::landmark_server::LandmarkServerNode)
