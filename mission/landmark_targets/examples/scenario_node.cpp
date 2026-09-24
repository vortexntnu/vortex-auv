// Example: how BT nodes use landmark_targets. Runs one RoboSub scenario against
// landmark_server (object_map) and waypoint_manager, without a behavior tree:
//
//   ros2 run landmark_targets landmark_targets_scenario_node
//     --ros-args -r __ns:=/nautilus -p scenario:=gate
//
// scenario: gate | slalom | torpedo | bin | return_home
// Ends with SCENARIO SUCCESS or SCENARIO FAILURE in the log and the matching
// exit code. The pieces map one to one to BT nodes:
//   ApproachStep    -> ApproachLandmark (LandmarkTarget + WaypointManager goal)
//   MoveRelativeStep-> MoveRelative (frame BODY_RELATIVE)
//   SlalomStep      -> Slalom (match_pipes, layer by layer)
//   ReturnHome      -> AvoidSlalom (course frame) + ApproachLandmark from
//   behind

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_track_array.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

#include "landmark_targets/geometry.hpp"
#include "landmark_targets/landmark_target.hpp"
#include "landmark_targets/ros_helpers.hpp"
#include "landmark_targets/slalom.hpp"

namespace lt = vortex::mission;
using WM = vortex_msgs::action::WaypointManager;
using LType = vortex_msgs::msg::LandmarkType;
using LSub = vortex_msgs::msg::LandmarkSubtype;
using vortex::utils::types::Pose;

enum class Status { RUNNING, SUCCESS, FAILURE };

/// Everything a step needs: the map, odometry, TF and the waypoint manager.
class Context {
   public:
    explicit Context(rclcpp::Node& node) : node_(node) {
        map_sub_ =
            node.create_subscription<vortex_msgs::msg::LandmarkTrackArray>(
                "landmark_server/object_map", rclcpp::QoS(10).reliable(),
                [this](vortex_msgs::msg::LandmarkTrackArray::ConstSharedPtr m) {
                    map_ = *m;
                });
        odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
            "odom", vortex::utils::qos_profiles::sensor_data_profile(1),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                odom_ = vortex::utils::ros_conversions::ros_pose_to_pose(
                    m->pose.pose);
            });
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node.get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        client_ = rclcpp_action::create_client<WM>(&node, "waypoint_manager");
        odom_frame_ =
            node.declare_parameter<std::string>("odom_frame", "nautilus/odom");
        base_frame_ = node.declare_parameter<std::string>("base_frame",
                                                          "nautilus/base_link");
        tool_frame_ = node.declare_parameter<std::string>("tool_frame", "");
        course_frame_ = node.declare_parameter<std::string>("course_frame",
                                                            "nautilus/course");
    }

    double now() const { return node_.now().seconds(); }
    const std::optional<Pose>& odom() const { return odom_; }
    tf2_ros::Buffer& tf() { return *tf_buffer_; }
    const std::string& odom_frame() const { return odom_frame_; }
    const std::string& course_frame() const { return course_frame_; }

    Eigen::Vector3d tool_arm() {
        if (tool_frame_.empty()) {
            return Eigen::Vector3d::Zero();
        }
        return lt::lookup_tool_arm(*tf_buffer_, base_frame_, tool_frame_)
            .value_or(Eigen::Vector3d::Zero());
    }

    /// Landmarks of a class from the map, as MapLandmarks.
    std::vector<lt::MapLandmark> landmarks(uint16_t type,
                                           uint16_t subtype) const {
        std::vector<lt::MapLandmark> out;
        if (!map_) {
            return out;
        }
        for (const auto& t : map_->landmark_tracks) {
            if (t.landmark.type.value == type &&
                t.landmark.subtype.value == subtype) {
                out.push_back(lt::map_landmark_from_track(t));
            }
        }
        return out;
    }

    // --- WaypointManager -----------------------------------------------------

    enum class NavState { IDLE, ACTIVE, SUCCEEDED, FAILED };

    /// Send a goal. A goal that replaces a running one is fine: the old one
    /// ends PREEMPTED and is ignored.
    void send(std::vector<vortex_msgs::msg::Waypoint> waypoints,
              uint8_t frame = WM::Goal::WORLD) {
        if (!client_->wait_for_action_server(std::chrono::milliseconds(100))) {
            nav_state_ = NavState::FAILED;
            nav_message_ = "waypoint_manager not available";
            return;
        }
        WM::Goal goal;
        goal.waypoints = std::move(waypoints);
        goal.frame = frame;
        goal.persistent = false;
        goal.convergence_threshold = 0.3;
        const uint64_t seq = ++goal_seq_;
        nav_state_ = NavState::ACTIVE;

        rclcpp_action::Client<WM>::SendGoalOptions options;
        options.goal_response_callback =
            [this, seq](rclcpp_action::ClientGoalHandle<WM>::SharedPtr gh) {
                if (seq != goal_seq_) {
                    return;
                }
                if (!gh) {
                    nav_state_ = NavState::FAILED;
                    nav_message_ = "goal rejected";
                }
                handle_ = gh;
            };
        options.result_callback =
            [this,
             seq](const rclcpp_action::ClientGoalHandle<WM>::WrappedResult& r) {
                if (seq != goal_seq_) {
                    return;  // replaced by a newer goal of ours
                }
                nav_message_ = r.result->message;
                nav_state_ = r.result->outcome == WM::Result::SUCCEEDED
                                 ? NavState::SUCCEEDED
                                 : NavState::FAILED;
                handle_.reset();  // finished: nothing left to cancel
            };
        client_->async_send_goal(goal, options);
    }

    NavState nav_state() const { return nav_state_; }
    const std::string& nav_message() const { return nav_message_; }

    void cancel() {
        ++goal_seq_;
        if (handle_) {
            try {
                client_->async_cancel_goal(handle_);
            } catch (const rclcpp_action::exceptions::UnknownGoalHandleError&) {
                // Already finished.
            }
            handle_.reset();
        }
        nav_state_ = NavState::IDLE;
    }

    /// A waypoint with the default tolerances and a hold, as a BT node would
    /// take them from its ports.
    static vortex_msgs::msg::Waypoint waypoint(
        const Pose& pose,
        uint8_t mode,
        double position_tol = 0.3,
        double orientation_tol_deg = 20.0,
        double hold_sec = 1.0) {
        vortex_msgs::msg::Waypoint wp;
        wp.pose = vortex::utils::ros_conversions::to_pose_msg(pose);
        wp.waypoint_mode.mode = mode;
        wp.position_tolerance = position_tol;
        wp.orientation_tolerance = orientation_tol_deg * M_PI / 180.0;
        wp.hold_time_sec = hold_sec;
        return wp;
    }

   private:
    rclcpp::Node& node_;
    rclcpp::Subscription<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::optional<vortex_msgs::msg::LandmarkTrackArray> map_;
    std::optional<Pose> odom_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<WM>::SharedPtr client_;
    rclcpp_action::ClientGoalHandle<WM>::SharedPtr handle_;
    uint64_t goal_seq_{0};
    NavState nav_state_{NavState::IDLE};
    std::string nav_message_;
    std::string odom_frame_, base_frame_, tool_frame_, course_frame_;
};

class Step {
   public:
    virtual ~Step() = default;
    virtual std::string name() const = 0;
    virtual void start(Context&) {}
    virtual Status tick(Context&) = 0;
    virtual void halt(Context& c) { c.cancel(); }
};

/// ApproachLandmark: lock the id, send goals while the target moves, stop at
/// dead reckoning, fail when the landmark is lost.
class ApproachStep : public Step {
   public:
    ApproachStep(std::string label,
                 uint16_t type,
                 uint16_t subtype,
                 lt::TargetSpec spec,
                 uint8_t mode)
        : label_(std::move(label)),
          type_(type),
          subtype_(subtype),
          spec_(std::move(spec)),
          mode_(mode) {}

    std::string name() const override { return "approach " + label_; }

    void start(Context& c) override {
        spec_.tool_arm = c.tool_arm();
        target_.reset();
        done_ = false;
    }

    Status tick(Context& c) override {
        if (!c.odom()) {
            return Status::RUNNING;
        }
        // Lock the id on the first landmark seen (the nearest, for several).
        auto candidates = c.landmarks(type_, subtype_);
        if (!target_) {
            if (candidates.empty()) {
                return timed_out(c) ? Status::FAILURE : Status::RUNNING;
            }
            const auto nearest = std::min_element(
                candidates.begin(), candidates.end(),
                [&](const auto& a, const auto& b) {
                    return (a.pose.pos_vector() - c.odom()->pos_vector())
                               .norm() <
                           (b.pose.pos_vector() - c.odom()->pos_vector())
                               .norm();
                });
            target_.emplace(spec_, nearest->id);
        }
        if (done_) {
            return finish(c);
        }

        std::optional<lt::MapLandmark> landmark;
        for (const auto& l : candidates) {
            if (l.id == target_->landmark_id()) {
                landmark = l;
            }
        }
        const auto step = target_->step(landmark, *c.odom(), c.now());
        if (step.phase == lt::Phase::LOST) {
            spdlog::error("{}: landmark lost", name());
            c.cancel();
            return Status::FAILURE;
        }
        if (step.send_goal) {
            c.send({Context::waypoint(*step.send_goal, mode_)});
        }
        if (step.phase == lt::Phase::DEAD_RECKONING) {
            done_ = true;  // keep the last goal; wait for it to finish
        }
        return finish(c);
    }

   private:
    Status finish(Context& c) {
        if (!done_) {
            // Under way: a failed goal is a failure, a reached one is not the
            // end while the target may still move.
            return c.nav_state() == Context::NavState::FAILED ? Status::FAILURE
                                                              : Status::RUNNING;
        }
        switch (c.nav_state()) {
            case Context::NavState::SUCCEEDED:
                return Status::SUCCESS;
            case Context::NavState::FAILED:
                spdlog::error("{}: {}", name(), c.nav_message());
                return Status::FAILURE;
            default:
                return Status::RUNNING;
        }
    }

    bool timed_out(Context& c) {
        if (!wait_started_) {
            wait_started_ = c.now();
        }
        return c.now() - *wait_started_ > 20.0;
    }

    std::string label_;
    uint16_t type_, subtype_;
    lt::TargetSpec spec_;
    uint8_t mode_;
    std::optional<lt::LandmarkTarget> target_;
    std::optional<double> wait_started_;
    bool done_{false};
};

/// MoveRelative: one offset in the vehicle frame at goal start.
class MoveRelativeStep : public Step {
   public:
    explicit MoveRelativeStep(Eigen::Vector3d offset)
        : offset_(std::move(offset)) {}
    std::string name() const override { return "move relative"; }
    void start(Context& c) override {
        sent_ = false;
        c.cancel();
    }
    Status tick(Context& c) override {
        if (!sent_) {
            c.send(
                {Context::waypoint(
                    Pose::from_eigen(offset_, Eigen::Quaterniond::Identity()),
                    vortex_msgs::msg::WaypointMode::ONLY_POSITION)},
                WM::Goal::BODY_RELATIVE);
            sent_ = true;
            return Status::RUNNING;
        }
        switch (c.nav_state()) {
            case Context::NavState::SUCCEEDED:
                return Status::SUCCESS;
            case Context::NavState::FAILED:
                return Status::FAILURE;
            default:
                return Status::RUNNING;
        }
    }

   private:
    Eigen::Vector3d offset_;
    bool sent_{false};
};

/// GoTo: odom waypoints (AvoidSlalom).
class GoToStep : public Step {
   public:
    using Provider = std::function<std::optional<std::vector<Pose>>(Context&)>;
    GoToStep(std::string label, Provider provider)
        : label_(std::move(label)), provider_(std::move(provider)) {}
    std::string name() const override { return "go to " + label_; }
    void start(Context& c) override {
        sent_ = false;
        c.cancel();
    }
    Status tick(Context& c) override {
        if (!sent_) {
            const auto poses = provider_(c);
            if (!poses) {
                return Status::FAILURE;
            }
            std::vector<vortex_msgs::msg::Waypoint> wps;
            for (const auto& p : *poses) {
                wps.push_back(Context::waypoint(
                    p, vortex_msgs::msg::WaypointMode::POSITION_AND_YAW));
            }
            c.send(std::move(wps));
            sent_ = true;
            return Status::RUNNING;
        }
        switch (c.nav_state()) {
            case Context::NavState::SUCCEEDED:
                return Status::SUCCESS;
            case Context::NavState::FAILED:
                return Status::FAILURE;
            default:
                return Status::RUNNING;
        }
    }

   private:
    std::string label_;
    Provider provider_;
    bool sent_{false};
};

/// Slalom: layer by layer with match_pipes.
class SlalomStep : public Step {
   public:
    SlalomStep(lt::Side gate_side, int layers, double z)
        : gate_side_(gate_side), layers_(layers), z_(z) {}
    std::string name() const override { return "slalom"; }

    void start(Context& c) override {
        passed_.clear();
        offset_.reset();
        layer_ = 0;
        phase_ = Phase::MATCH;
        c.cancel();
        started_ = c.now();
    }

    Status tick(Context& c) override {
        if (!c.odom()) {
            return Status::RUNNING;
        }
        if (layer_ >= layers_) {
            return Status::SUCCESS;
        }
        switch (phase_) {
            case Phase::MATCH: {
                std::vector<lt::Pipe> red, white;
                for (const auto& l :
                     c.landmarks(LType::SLALOM_PIPE, LSub::SLALOM_PIPE_RED)) {
                    red.push_back({l.id, l.pose.pos_vector()});
                }
                for (const auto& l :
                     c.landmarks(LType::SLALOM_PIPE, LSub::SLALOM_PIPE_WHITE)) {
                    white.push_back({l.id, l.pose.pos_vector()});
                }
                const auto gap = lt::match_pipes(red, white, *c.odom(),
                                                 gate_side_, passed_, offset_);
                if (!gap) {
                    // A layer with no pipes in sight: go blind, 2 m straight
                    // ahead.
                    if (c.now() - started_ > 10.0) {
                        spdlog::warn("slalom: no pipes matched, blind layer");
                        move_blind(c);
                    }
                    return Status::RUNNING;
                }
                current_gap_ = *gap;
                const Eigen::Quaterniond q(
                    Eigen::AngleAxisd(gap->heading, Eigen::Vector3d::UnitZ()));
                c.send({Context::waypoint(
                    Pose::from_eigen(Eigen::Vector3d(gap->position.x(),
                                                     gap->position.y(), z_),
                                     q),
                    vortex_msgs::msg::WaypointMode::POSITION_AND_YAW)});
                phase_ = Phase::GO_TO_GAP;
                return Status::RUNNING;
            }
            case Phase::GO_TO_GAP:
                if (c.nav_state() == Context::NavState::SUCCEEDED) {
                    // Through the gap, then remember the layer.
                    c.send({Context::waypoint(
                               Pose::from_eigen(Eigen::Vector3d(1.0, 0.0, 0.0),
                                                Eigen::Quaterniond::Identity()),
                               vortex_msgs::msg::WaypointMode::ONLY_POSITION)},
                           WM::Goal::BODY_RELATIVE);
                    phase_ = Phase::PASS;
                } else if (c.nav_state() == Context::NavState::FAILED) {
                    return Status::FAILURE;
                }
                return Status::RUNNING;
            case Phase::PASS:
                if (c.nav_state() == Context::NavState::SUCCEEDED) {
                    last_reference_ = c.odom()->pos_vector().head<2>();
                    if (current_gap_.red_id >= 0) {
                        // A matched layer: remember it for the next one. A
                        // blind layer teaches nothing about the offset.
                        passed_.push_back(current_gap_.red_id);
                        offset_ = lt::SlalomOffset{current_gap_.offset_from_red,
                                                   current_gap_.heading};
                    }
                    ++layer_;
                    phase_ = Phase::MATCH;
                    started_ = c.now();
                } else if (c.nav_state() == Context::NavState::FAILED) {
                    return Status::FAILURE;
                }
                return Status::RUNNING;
        }
        return Status::RUNNING;
    }

    std::optional<Eigen::Vector2d> last_reference() const {
        return last_reference_;
    }

   private:
    enum class Phase { MATCH, GO_TO_GAP, PASS };

    void move_blind(Context& c) {
        c.send(
            {Context::waypoint(Pose::from_eigen(Eigen::Vector3d(2.0, 0.0, 0.0),
                                                Eigen::Quaterniond::Identity()),
                               vortex_msgs::msg::WaypointMode::ONLY_POSITION)},
            WM::Goal::BODY_RELATIVE);
        phase_ = Phase::PASS;
        current_gap_.red_id = -1;
    }

    lt::Side gate_side_;
    int layers_;
    double z_;
    int layer_{0};
    Phase phase_{Phase::MATCH};
    std::vector<int> passed_;
    std::optional<lt::SlalomOffset> offset_;
    lt::PipeGap current_gap_;
    std::optional<Eigen::Vector2d> last_reference_;
    double started_{0.0};
};

class ScenarioNode : public rclcpp::Node {
   public:
    ScenarioNode() : Node("landmark_targets_scenario_node"), ctx_(*this) {
        const auto scenario =
            declare_parameter<std::string>("scenario", "gate");
        const auto role =
            declare_parameter<std::string>("role", "survey_repair");
        const bool survey = role == "survey_repair";
        const double depth = declare_parameter<double>("depth", 2.0);
        const auto gate_side =
            declare_parameter<std::string>("gate_side", "left");
        const auto side =
            gate_side == "left" ? lt::Side::LEFT : lt::Side::RIGHT;
        // Lane limits in course y (y to the right).
        const double lane_y_min = declare_parameter<double>("lane_y_min", -6.0);
        const double lane_y_max = declare_parameter<double>("lane_y_max", 6.0);

        const auto yaw_pose = [](double x, double y, double z, double yaw) {
            return Pose::from_eigen(Eigen::Vector3d(x, y, z),
                                    Eigen::Quaterniond(Eigen::AngleAxisd(
                                        yaw, Eigen::Vector3d::UnitZ())));
        };
        using WMode = vortex_msgs::msg::WaypointMode;

        if (scenario == "gate") {
            lt::TargetSpec spec;
            spec.frame = lt::OffsetFrame::LANDMARK;
            // Through the opening of the chosen role, not the gate centre:
            // the middle post hangs there. The role image sits high in its
            // opening, so go 0.45 m deeper to pass through the middle of it.
            spec.offset =
                yaw_pose(2.5, 0.0, 0.45, M_PI);  // in front, facing it
            const auto sub =
                survey ? LSub::GATE_SURVEY_REPAIR : LSub::GATE_SEARCH_RESCUE;
            steps_.push_back(std::make_unique<ApproachStep>(
                "gate opening", LType::GATE, sub, spec, WMode::FULL_POSE));
            steps_.push_back(std::make_unique<MoveRelativeStep>(
                Eigen::Vector3d(5.0, 0.0, 0.0)));
        } else if (scenario == "torpedo") {
            lt::TargetSpec spec;
            spec.frame = lt::OffsetFrame::LANDMARK;
            spec.offset = yaw_pose(1.5, 0.0, 0.0, M_PI);
            spec.dead_reckoning_distance = 0.3;
            const auto sub = survey ? LSub::TORPEDO_TARGET_LARGE_SURVEY_REPAIR
                                    : LSub::TORPEDO_TARGET_LARGE_SEARCH_RESCUE;
            steps_.push_back(std::make_unique<ApproachStep>(
                "torpedo opening", LType::TORPEDO_BOARD, sub, spec,
                WMode::FULL_POSE));
        } else if (scenario == "bin") {
            lt::TargetSpec spec;
            spec.frame = lt::OffsetFrame::LANDMARK_ODOM_AXES;
            spec.offset = yaw_pose(0.0, 0.0, -1.0, 0.0);  // 1 m above the bin
            spec.dead_reckoning_distance = 0.3;
            const auto sub =
                survey ? LSub::BIN_SURVEY_REPAIR : LSub::BIN_SEARCH_RESCUE;
            steps_.push_back(std::make_unique<ApproachStep>(
                "bin", LType::BIN, sub, spec, WMode::ONLY_POSITION));
        } else if (scenario == "slalom") {
            steps_.push_back(std::make_unique<SlalomStep>(side, 3, depth));
        } else if (scenario == "return_home") {
            // The course frame must be locked to the gate first.
            auto slalom_reference = declare_parameter<std::vector<double>>(
                "slalom_reference_xy", {0.0, 0.0});
            steps_.push_back(std::make_unique<GoToStep>(
                "around the slalom",
                [=](Context& c) -> std::optional<std::vector<Pose>> {
                    const auto course = lt::course_frame_from_tf(
                        c.tf(), c.odom_frame(), c.course_frame(),
                        lt::CourseState::GATE_LOCKED);
                    if (!course) {
                        spdlog::error(
                            "return_home: no course frame (state UNSET)");
                        return std::nullopt;
                    }
                    return lt::avoid_slalom_waypoints(
                        *course,
                        Eigen::Vector2d(slalom_reference[0],
                                        slalom_reference[1]),
                        lane_y_min, lane_y_max, 2.5, 0.8);
                }));
            lt::TargetSpec spec;
            spec.frame = lt::OffsetFrame::LANDMARK;
            spec.offset = yaw_pose(
                -2.0, 0.0, 0.0, 0.0);  // 2 m behind the gate, facing through it
            spec.dead_reckoning_distance = 1.5;
            steps_.push_back(std::make_unique<ApproachStep>(
                "gate from behind", LType::GATE, LSub::GATE_WHOLE, spec,
                WMode::FULL_POSE));
            steps_.push_back(std::make_unique<MoveRelativeStep>(
                Eigen::Vector3d(3.0, 0.0, 0.0)));
        } else {
            spdlog::error("unknown scenario '{}'", scenario);
            std::exit(2);
        }

        timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                   [this]() { tick(); });
        spdlog::info("scenario '{}' with {} step(s)", scenario, steps_.size());
    }

   private:
    void tick() {
        if (index_ >= steps_.size()) {
            return;
        }
        Step& step = *steps_[index_];
        if (!started_) {
            spdlog::info("step: {}", step.name());
            step.start(ctx_);
            started_ = true;
        }
        switch (step.tick(ctx_)) {
            case Status::RUNNING:
                return;
            case Status::SUCCESS:
                spdlog::info("step done: {}", step.name());
                ++index_;
                started_ = false;
                if (index_ >= steps_.size()) {
                    spdlog::info("SCENARIO SUCCESS");
                    std::exit(0);
                }
                return;
            case Status::FAILURE:
                step.halt(ctx_);
                spdlog::error("SCENARIO FAILURE in step: {}", step.name());
                std::exit(1);
        }
    }

    Context ctx_;
    std::vector<std::unique_ptr<Step>> steps_;
    std::size_t index_{0};
    bool started_{false};
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScenarioNode>());
    rclcpp::shutdown();
    return 0;
}
