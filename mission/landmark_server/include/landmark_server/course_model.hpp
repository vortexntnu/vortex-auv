#ifndef LANDMARK_SERVER__COURSE_MODEL_HPP_
#define LANDMARK_SERVER__COURSE_MODEL_HPP_

#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <functional>
#include <map>
#include <optional>
#include <pose_filtering/lib/typedefs.hpp>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include "landmark_server/structures.hpp"

namespace vortex::mission {

struct RetainedLandmark;

/// Classes a detector mixes up (a white pipe called red, the two red role
/// icons). They are tracked as one kind, the first class of the group; the
/// class a part gets comes from the template and the votes.
struct ClassGroup {
    std::string name;
    std::vector<LandmarkClassKey> classes;
};

/**
 * @brief One task of the course: a template placed at a prior pose in the
 * course frame (origin at the gate, x through the gate, y right).
 */
struct TaskSpec {
    std::string name;
    std::string template_name;
    /// Prior position (xy) and yaw of the task frame's +X, course frame.
    Eigen::Vector2d prior_xy{Eigen::Vector2d::Zero()};
    double prior_yaw{0.0};
    /// Before the task is placed: its parts are taken within this distance
    /// (xy) of the prior position [m].
    double region_radius_m{3.0};
    /// After it is placed: within this distance of the part they would be
    /// [m].
    double part_radius_m{1.0};
    /// The placement's yaw must be within this of the prior yaw [rad].
    double yaw_window_rad{M_PI};
    /// The template looks the same turned 180 deg (a slalom set).
    bool symmetric{false};
    /// Parts needed to place the task. 1: the part alone, at the prior yaw.
    int min_parts{2};
    /// Detections of its parts farther than this from the vehicle are not
    /// taken [m] (0 = any range): far detections of small parts are noisy
    /// and biased, and a part is kept for the whole run.
    double max_range_m{0.0};
};

struct CourseConfig {
    /// Off: every class is mapped as a free landmark (no course layout).
    bool enable{false};
    /// The vehicle's start position in the course frame: the course frame
    /// is first set at the start pose (origin there), then moved to the
    /// gate; the priors are relative to the gate.
    Eigen::Vector2d start_xy{Eigen::Vector2d::Zero()};
    std::vector<StructureTemplate> templates;
    std::vector<TaskSpec> tasks;
    std::vector<ClassGroup> class_groups;
    /// Tracker limit per kind: the parts of that kind in the course plus
    /// this many (a false track must not block a real part for long).
    int extra_tracks_per_kind{2};
    /// The variant of a template (torpedo decal version, gate panel sides)
    /// is decided when one has this many agreeing detections ...
    int variant_votes{40};
    /// ... and this many times more than the next.
    double variant_ratio{3.0};
    /// A part is never placed farther than this from where the template
    /// puts it, whatever its covariance says [m] (0.5 x the distance to the
    /// next part when that is smaller, at least min_slot_gate_m).
    double min_slot_gate_m{0.2};
    /// To place a task, the parts' reported classes must agree with the
    /// template at least this much (parts with >= 5 votes).
    double min_class_agreement{0.5};
    /// A track fills a part (or places a task) only after this many
    /// detections: a part is kept for the whole run, a few far, noisy
    /// detections must not decide where it is.
    int min_part_detections{8};

    const TaskSpec* task(const std::string& name) const;
    const StructureTemplate* template_for(const TaskSpec& t) const;
    /// The kind a class is tracked as.
    LandmarkClassKey kind_of(const LandmarkClassKey& key) const;
    /// Every kind that is a part of some task.
    std::set<std::pair<uint16_t, uint16_t>> templated_kinds() const;
};

/**
 * @brief Parse the `course` tree: enable, start, class_groups, templates
 * (as rules.structures: members or variants), tasks (template, prior
 * [x, y, yaw_deg], region_radius_m, part_radius_m, yaw_window_deg,
 * symmetric, min_parts).
 * @throws std::runtime_error with the key on errors.
 */
CourseConfig parse_course_config(const YAML::Node& node);

/// How the course frame maps to odom, from CourseFrameTracker.
struct CourseGeometry {
    bool set{false};
    /// Course frame -> odom (xy), and the course direction in odom.
    std::function<Eigen::Vector2d(const Eigen::Vector2d&)> to_odom;
    std::function<Eigen::Vector2d(const Eigen::Vector2d&)> to_course;
    double through_yaw{0.0};
    /// The course frame origin is the gate (else the start pose).
    bool at_gate{false};
};

/// What one confirmed track offers the course model.
struct KindTrack {
    int id{-1};
    LandmarkClassKey kind;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity() * 0.01};
};

/// Votes of this tick: track id -> (reported class -> detections).
using TrackVotes =
    std::map<int, std::map<std::pair<uint16_t, uint16_t>, int>>;

/**
 * @brief The course as a set of tasks with fixed parts (slots). ROS-free.
 *
 * Every task is a template (rigid arrangement of classes) at a prior pose
 * in the course frame. The map holds one landmark per slot, with a stable
 * id, never more: a detection that fits no slot is not an object.
 *
 * Each tick:
 *  - a placed task keeps the tracks on its slots while they stay within the
 *    slot gate, gives free slots the nearest unclaimed track of their kind
 *    within the gate (a remembered slot keeps its id: the smoothing graph
 *    sees the part again), and refits its pose to its slots (yaw within the
 *    window);
 *  - a task not placed yet fits its template to the tracks of its kinds
 *    within its region (prior moved by the offset of the nearest placed
 *    task), with the yaw window, at least min_parts parts and classes that
 *    agree with the template;
 *  - the variant follows the votes and is fixed once clear.
 *
 * Focus (from the mission): tasks outside the focus can be locked: no new
 * parts, no refit, they only move with the graph correction. A committed
 * task keeps its pose and variant.
 */
class CourseModel {
   public:
    struct Slot {
        std::string name;
        /// Index of the member in each variant, and its kind there.
        std::vector<std::size_t> member;
        std::vector<LandmarkClassKey> kinds;
        double gate_m{0.5};
        int landmark_id{-1};
        int track_id{-1};
        /// Reported classes of the tracks this part had before the one it
        /// has now (see part_votes).
        std::map<std::pair<uint16_t, uint16_t>, int> past_votes;
    };
    struct Task {
        const TaskSpec* spec{nullptr};
        const StructureTemplate* tmpl{nullptr};
        bool placed{false};
        Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
        std::size_t variant{0};
        bool variant_fixed{false};
        bool committed{false};
        std::vector<Slot> slots;
    };

    /// What the model needs from the landmark store.
    struct Store {
        std::function<RetainedLandmark*(int id)> find;
        std::function<RetainedLandmark&(const LandmarkClassKey& key,
                                        const std::string& slot)>
            create;
        /// Follow a track (position, covariance, observations).
        std::function<void(RetainedLandmark&, int track_id)> follow;
    };

    explicit CourseModel(CourseConfig config = {});

    /// New layout (restart / clear): all tasks unplaced.
    void reset(CourseConfig config);
    void clear();
    const CourseConfig& config() const { return config_; }
    bool enabled() const { return config_.enable; }

    /**
     * @brief One tick.
     * @param tracks Confirmed tracks of templated kinds.
     * @return The ids of the tracks the course uses.
     */
    std::set<int> update(const std::vector<KindTrack>& tracks,
                         const TrackVotes& votes,
                         const CourseGeometry& geo,
                         Store& store);

    /// The odom frame moved under the map (graph correction).
    void apply_correction(const Eigen::Isometry3d& delta);

    /// Intake: may a detection of this kind at this position go to the
    /// tracker? Empty = yes, else the reason it is dropped.
    std::optional<std::string> intake_reject(
        const LandmarkClassKey& kind,
        const Eigen::Vector3d& position,
        const CourseGeometry& geo,
        const std::optional<Eigen::Vector3d>& vehicle = std::nullopt) const;

    /// Focus: tasks the mission works on (empty = all), others frozen when
    /// lock_others. Unknown names give an error message.
    std::optional<std::string> set_focus(const std::vector<std::string>& tasks,
                                         bool lock_others);
    std::optional<std::string> commit(const std::string& task, bool on);
    const std::vector<std::string>& focus() const { return focus_; }
    bool lock_others() const { return lock_others_; }
    bool in_focus(const Task& t) const;
    bool locked(const Task& t) const;

    const std::vector<Task>& tasks() const { return tasks_; }
    /// Where the template puts each slot now (odom); for a task not placed,
    /// at its prior.
    Eigen::Vector3d slot_position(const Task& t, std::size_t slot) const;
    /// The class a slot has now (variant, or the vote among its classes).
    LandmarkClassKey slot_class(const Task& t, std::size_t slot) const;
    /// Reported classes of everything this part was seen as: its past
    /// tracks and the whole life of its track now (from its first
    /// detection, before it was confirmed).
    std::map<std::pair<uint16_t, uint16_t>, int> part_votes(const Task& t,
                                                            std::size_t slot) const;
    /// Whether a track of this kind can be the slot's part: of the variant
    /// in use once it is fixed, of any variant before.
    bool slot_accepts(const Task& t, std::size_t slot, const LandmarkClassKey& kind) const;
    /// Task pose (odom) used now: placed pose, else the prior moved by the
    /// offset of the nearest placed task. Empty without a course frame.
    std::optional<Eigen::Isometry3d> working_pose(const Task& t,
                                                  const CourseGeometry& geo) const;
    std::string variant_name(const Task& t) const;
    /// Tracker limit per kind (parts in the course + extra).
    std::map<std::pair<uint16_t, uint16_t>, int> track_limits() const;

   private:
    void build_tasks();
    Eigen::Isometry3d prior_pose(const Task& t, const CourseGeometry& geo) const;
    /// Course-frame offset (placed - prior) of the placed task nearest to
    /// @p t's prior, or zero.
    Eigen::Vector2d neighbour_offset(const Task& t,
                                     const CourseGeometry& geo) const;
    void place(Task& t,
               const std::vector<KindTrack>& tracks,
               std::set<int>& claimed,
               const CourseGeometry& geo,
               Store& store);
    void keep_and_fill(Task& t,
                       const std::vector<KindTrack>& tracks,
                       std::set<int>& claimed,
                       Store& store);
    void refit(Task& t, const CourseGeometry& geo, Store& store);
    void decide_variant(Task& t);
    void attach(Task& t, std::size_t slot, int track_id, Store& store);
    double class_agreement(const Task& t,
                           const std::vector<std::pair<std::size_t, int>>& parts,
                           const TrackVotes& votes) const;

    CourseConfig config_;
    std::vector<Task> tasks_;
    std::vector<std::string> focus_;
    bool lock_others_{false};
    /// Reported classes per track, summed over its life (also before it is
    /// confirmed), and the tick it last had a detection.
    TrackVotes track_votes_;
    std::map<int, int> track_last_vote_;
    int tick_{0};
    /// A part lets go of its track: its votes stay with the part.
    void release(Slot& s);
    /// Detections associated to a track over its life.
    int detections(int track_id) const;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__COURSE_MODEL_HPP_
