#pragma once

#include <Eigen/Dense>
#include <vector>
#include <vortex_filtering/filters/ipda.hpp>
#include <vortex_filtering/vortex_filtering.hpp>

using State2d = vortex::prob::Gauss<2>;
using DynMod = vortex::models::ConstantPosition;
using SensorMod = vortex::models::IdentitySensorModel<2, 2>;
using IPDA = vortex::filter::IPDA<DynMod, SensorMod>;

struct Track {
    int id;
    State2d state;
    double existence_probability;
    bool confirmed;
    Eigen::Matrix<double, 2, 2> line_points;
    int id1;
    int id2;

    // For sorting tracks based on existence probability and confirmed track
    bool operator<(const Track& other) const {
        if (confirmed != other.confirmed) {
            return confirmed > other.confirmed;
        } else {
            return existence_probability > other.existence_probability;
        }
    }

    bool operator==(const Track& other) const {
        return confirmed == other.confirmed &&
               existence_probability == other.existence_probability;
    }
};

class TrackManager {
   public:
    /**
     * @brief Default constructor for the TrackManager class.
     */
    TrackManager();

    /**
     * @brief Updates the tracks based on the measurements received from the
     * sensor.
     *
     * @param measurements The measurements received from the sensor.
     * @param update_interval The time interval between updates.
     * @param confirmation_threshold The threshold for confirming a track.
     * @param gate_theshhold The threshold for gating measurements.
     * @param prob_of_detection The probability of detection.
     * @param prob_of_survival The probability of survival.
     * @param clutter_intensity The intensity of clutter.
     */
    void update_line_tracks(Eigen::Array<double, 2, Eigen::Dynamic> measurements,
                      Eigen::Array<double, 2, Eigen::Dynamic> line_params,
                      int update_interval,
                      double confirmation_threshold,
                      double gate_theshhold,
                      double min_gate_threshold,
                      double max_gate_threshold,
                      double prob_of_detection,
                      double prob_of_survival,
                      double clutter_intensity,
                      double initial_existence_probability);

    void update_line_intersection_tracks(Eigen::Array<double, 2, Eigen::Dynamic> intersections,
                    Eigen::Array<int, 2, Eigen::Dynamic> current_intersection_ids,
                    Eigen::Array<double, 2, Eigen::Dynamic> current_line_intersection_points,
                    int update_interval, 
                    double confirmation_threshold, 
                    double gate_theshhold, 
                    double min_gate_threshold, 
                    double max_gate_threshold, 
                    double prob_of_detection, 
                    double prob_of_survival, 
                    double clutter_intensity,
                    double initial_existence_probability);

    /**
     * @brief Creates new tracks for every measurements.
     *
     * @param measurements The measurements received.
     */
    void create_line_tracks(Eigen::Array<double, 2, Eigen::Dynamic> measurements,
                      Eigen::Array<double, 2, Eigen::Dynamic> line_params,
                      double initial_existence_probability);

    void create_line_intersection_tracks(Eigen::Array<double, 2, Eigen::Dynamic> intersections,
                    Eigen::Array<int, 2, Eigen::Dynamic> current_intersection_ids,
                    Eigen::Array<double, 2, Eigen::Dynamic> current_line_intersection_points,
                    double initial_existence_probability);

    /**
     * @brief Deletes tracks that have a low probability of existence.
     *
     * @param deletion_threshold The threshold for deleting a track.
     */
    void delete_tracks(double deletion_threshold);

    /**
     * @brief Sets the dynamic model for estimating target motion.
     *
     * @param std_velocity The standard deviation of the target velocity.
     */
    void set_dyn_model(double std_velocity);

    /**
     * @brief Sets the sensor model for estimating target measurements.
     *
     * @param std_measurement The standard deviation of the target measurement.
     */
    void set_sensor_model(double std_measurement);

    /**
     * @brief Retrieves the current tracks.
     *
     * @return A vector of Track objects representing the current tracks.
     */
    std::vector<Track> get_tracks() const { return tracks_; }

    void delete_track_by_id(int id);

   private:
    std::vector<Track> tracks_;  ///< The vector of tracks.

    std::shared_ptr<DynMod>
        dyn_model_;  ///< The dynamic model for estimating target motion.

    std::shared_ptr<SensorMod>
        sensor_model_;  ///< The sensor model for estimating target
                        ///< measurements.

    int tracker_id_;  ///< The tracker id.
};
