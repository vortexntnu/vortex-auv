#ifndef LANDMARK_SERVER__RETAINED_LANDMARKS_HPP_
#define LANDMARK_SERVER__RETAINED_LANDMARKS_HPP_

#include <deque>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <map>
#include <pose_filtering/lib/typedefs.hpp>
#include <string>
#include <vector>
#include "landmark_server/class_config.hpp"
#include "landmark_server/course_model.hpp"

namespace vortex::mission {

/**
 * @brief A landmark in the map: what the tree can rely on after the live
 * tracker has forgotten it. Stable id, alive for as long as the class rules
 * say. ROS-free.
 */
struct RetainedLandmark {
    /// Stable, never reused.
    int id{-1};
    vortex::filtering::LandmarkClassKey key{};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    /// +X out of the front, +Z down (NED). Valid when has_orientation.
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    bool has_orientation{false};
    /// The yaw is fixed (by the course model) and no longer follows the
    /// tracker.
    bool yaw_locked{false};
    /// Computed by a map rule instead of measured.
    bool derived{false};
    Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};
    /// Seconds, same clock as the `now` given to update().
    double first_seen{0.0};
    double last_measurement{0.0};
    int observations{0};
    /// Id of the live track this landmark follows; -1 = remembered only.
    int live_track_id{-1};
    int hits{0};
    int misses{0};

    /// Stable name of a derived landmark ("octagon_whole", a course task's
    /// point), so it is the same landmark every tick.
    std::string derived_slot;
    /// The course slot this landmark fills ("slalom_1/red"), or empty for a
    /// free landmark. Slot landmarks are kept for the rest of the run.
    std::string course_slot;
    /// Hidden from the published map because another landmark describes the
    /// same object better (id of that landmark), or -1.
    int absorbed_by{-1};

    /// For a derived landmark: the parts it is computed from are being seen.
    bool derived_live{false};

    double yaw() const;
    /// Seen now: followed by a live track, or derived from parts that are.
    bool is_live() const {
        return live_track_id >= 0 || (derived && derived_live);
    }
};

/**
 * @brief Turns the live tracks from PoseTrackManager into a map with stable
 * ids and memory, following the class rules (max instances, adoption radius,
 * retention). ROS-free.
 *
 * With a course layout (config `course`), the classes that are parts of a
 * task are mapped only by the course model: one landmark per slot, never
 * forgotten, no landmark outside a slot (see CourseModel). The rest below
 * applies to the other classes (free landmarks).
 *
 * Live track -> landmark:
 *  - a track that is already followed updates its landmark;
 *  - a new track takes over the nearest remembered landmark of the same class
 *    within the adoption radius (so the id survives a track being deleted and
 *    recreated), but only when that landmark is clearly the nearest (the next
 *    one of the class adoption_ambiguity_ratio times farther away). An
 *    ambiguous track waits up to adoption_wait_sec for that; a wrong take-over
 *    would join two objects in the smoothing graph for the rest of the run;
 *  - otherwise it becomes a new landmark, unless the class is full or it
 *    lies outside the lane.
 */
class RetainedLandmarks {
   public:
    using PositionFilter = std::function<bool(const Eigen::Vector3d&)>;

    explicit RetainedLandmarks(LandmarkMapConfig config);

    /// What the course model needs each tick besides the tracks.
    struct CourseInput {
        TrackVotes votes;
        CourseGeometry geometry;
    };

    /**
     * @param confirmed The confirmed tracks of PoseTrackManager.
     * @param now Time [s].
     * @param position_allowed Optional lane; landmarks outside are
     * rejected (new) or dropped (existing).
     * @param course Class votes per track and the course frame.
     */
    void update(const std::vector<vortex::filtering::Track>& confirmed,
                double now,
                const PositionFilter& position_allowed = {},
                const CourseInput& course = {});

    /// Forget everything. Ids keep counting up.
    void clear();

    /// New rules (live parameter change). The landmarks stay; the new rules
    /// apply from the next update (a lower max_instances does not remove
    /// landmarks that are already there).
    /// The course layout is read at start: a change of `course` here is
    /// ignored.
    void set_config(LandmarkMapConfig config) { config_ = std::move(config); }

    /// The course model (tasks and their slots).
    const CourseModel& course() const { return course_; }
    CourseModel& course() { return course_; }

    /**
     * @brief The odom frame moved under the map (the smoothing graph changed
     * its correction by @p delta, new_odom <- old_odom). Everything kept in
     * odom coordinates that is not set again this tick moves with it: every
     * orientation (a locked yaw included) and the positions of remembered
     * landmarks that the graph does not place (@p placed_by_graph false).
     * Followed landmarks keep their fresh tracker position.
     */
    void apply_correction(const Eigen::Isometry3d& delta,
                          const std::function<bool(int)>& placed_by_graph);

    const std::deque<RetainedLandmark>& landmarks() const { return landmarks_; }
    /// For the map rules, which may change orientation and add derived
    /// landmarks.
    std::deque<RetainedLandmark>& landmarks() { return landmarks_; }

    const RetainedLandmark* find(int id) const;

    /**
     * @brief The derived landmark with this slot, created on first use.
     * Derived landmarks keep their id and are remembered for the rest of the
     * run. References stay valid when others are added.
     */
    RetainedLandmark& upsert_derived(
        const std::string& slot,
        const vortex::filtering::LandmarkClassKey& key,
        double now);

    /// Number of tracks rejected because a class was full or out of the
    /// lane.
    int rejected_count() const { return rejected_; }

    /// Number of ticks a track waited because two landmarks were about as
    /// near (ambiguous take-over).
    int ambiguous_count() const { return ambiguous_; }

   private:
    void update_from_track(RetainedLandmark& lm,
                           const vortex::filtering::Track& track,
                           double now) const;
    void forget_expired(double now);

    /// The course model's tracks and slots this tick.
    void update_course(const std::vector<vortex::filtering::Track>& confirmed,
                       double now,
                       const CourseInput& input);

    LandmarkMapConfig config_;
    std::deque<RetainedLandmark> landmarks_;
    CourseModel course_;
    int next_id_{0};
    int rejected_{0};
    int ambiguous_{0};
    /// Tracks waiting for an ambiguous take-over to resolve: track id -> time
    /// the wait started.
    std::map<int, double> ambiguous_since_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__RETAINED_LANDMARKS_HPP_
