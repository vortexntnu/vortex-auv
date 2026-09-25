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
    /// The yaw is fixed by the map rules and no longer follows the tracker.
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

    /// Stable name of a derived landmark ("gate_whole", "torpedo_target_fire"
    /// ...), so a rule updates the same landmark every tick.
    std::string derived_slot;
    /// Hidden from the published map because another landmark describes the
    /// same object better (id of that landmark), or -1.
    int absorbed_by{-1};
    /// Running circular mean of consistent yaw estimates (map rules).
    double yaw_sum_sin{0.0};
    double yaw_sum_cos{0.0};
    int yaw_count{0};

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
 * Live track -> landmark:
 *  - a track that is already followed updates its landmark;
 *  - a new track takes over the nearest remembered landmark of the same class
 *    within the adoption radius (so the id survives a track being deleted and
 *    recreated), but only when that landmark is clearly the nearest (the next
 *    one of the class adoption_ambiguity_ratio times farther away). An
 *    ambiguous track waits up to adoption_wait_sec for that; a wrong take-over
 *    would join two objects in the smoothing graph for the rest of the run;
 *  - otherwise it becomes a new landmark, unless the class is full, it lies
 *    outside the lane bounds, or (for pipes) too close to a large structure.
 */
class RetainedLandmarks {
   public:
    using PositionFilter = std::function<bool(const Eigen::Vector3d&)>;

    explicit RetainedLandmarks(LandmarkMapConfig config);

    /**
     * @param confirmed The confirmed tracks of PoseTrackManager.
     * @param now Time [s].
     * @param position_allowed Optional lane bounds; landmarks outside are
     * rejected (new) or dropped (existing).
     */
    void update(const std::vector<vortex::filtering::Track>& confirmed,
                double now,
                const PositionFilter& position_allowed = {});

    /// Forget everything. Ids keep counting up.
    void clear();

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

    /// Number of tracks rejected because a class was full, out of bounds or
    /// too close to a large structure.
    int rejected_count() const { return rejected_; }

    /// Number of ticks a track waited because two landmarks were about as
    /// near (ambiguous take-over).
    int ambiguous_count() const { return ambiguous_; }

   private:
    void update_from_track(RetainedLandmark& lm,
                           const vortex::filtering::Track& track,
                           double now) const;
    bool near_large_structure(const Eigen::Vector3d& position,
                              double distance) const;
    void forget_expired(double now);

    LandmarkMapConfig config_;
    std::deque<RetainedLandmark> landmarks_;
    int next_id_{0};
    int rejected_{0};
    int ambiguous_{0};
    /// Tracks waiting for an ambiguous take-over to resolve: track id -> time
    /// the wait started.
    std::map<int, double> ambiguous_since_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__RETAINED_LANDMARKS_HPP_
