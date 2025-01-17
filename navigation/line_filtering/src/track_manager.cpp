#include <line_filtering/track_manager.hpp>

TrackManager::TrackManager()
: tracker_id_(0)
{
}

void TrackManager::updateTracks(Eigen::Array<double, 2, Eigen::Dynamic> measurements,
    Eigen::Array<double, 2, Eigen::Dynamic> line_params,
    int update_interval, 
    double confirmation_threshold, 
    double gate_theshhold, 
    double min_gate_threshold, 
    double max_gate_threshold, 
    double prob_of_detection, 
    double prob_of_survival, 
    double clutter_intensity,
    double initial_existence_probability)
{
    // Sorts the tracks based on existence probability and confirmed track
    std::sort(tracks_.begin(), tracks_.end());

    for (auto &track : tracks_)
    {
        IPDA::Config config;
        config.pdaf.mahalanobis_threshold = gate_theshhold;
        config.pdaf.min_gate_threshold = min_gate_threshold;
        config.pdaf.max_gate_threshold = max_gate_threshold;
        config.pdaf.prob_of_detection = prob_of_detection;
        config.pdaf.clutter_intensity = clutter_intensity;
        config.ipda.prob_of_survival = prob_of_survival;
        config.ipda.estimate_clutter = false;
        config.pdaf.clutter_intensity = clutter_intensity;

        IPDA::State state_est_prev;
        state_est_prev.x_estimate = track.state;
        state_est_prev.existence_probability = track.existence_probability;
        // Predict next state
        auto output = 
            IPDA::step(*dyn_model_, 
            *sensor_model_,
            update_interval / 1000.0,
            state_est_prev, 
            line_params, 
            config);
        // Update state
        track.state = output.state.x_estimate;
        // Update existence probability
        track.existence_probability = output.state.existence_probability;

        // Update track existence
        if (track.confirmed == false && output.state.existence_probability > confirmation_threshold)
        {
            track.confirmed = true;
        }

        // Update the measurement list
        Eigen::Array<double, 2, Eigen::Dynamic> outside(2, line_params.cols());
        Eigen::Array<double, 2, Eigen::Dynamic> outside_measurements(2, measurements.cols());

        Eigen::Index inside_num = 0;
        for (Eigen::Index i = 0; i < line_params.cols(); ++i)
        {
            if (output.gated_measurements[i])
            {
                track.line_points = measurements.block<2, 2>(0, i*2);
                inside_num++;
            }
            else
            {
                outside.col(i-inside_num) = line_params.col(i);
                outside_measurements.block<2, 2>(0, 2 * (i - inside_num)) = measurements.block<2, 2>(0, i * 2);
            }
        }
        outside.conservativeResize(2, line_params.cols() - inside_num);
        outside_measurements.conservativeResize(2, measurements.cols() - inside_num*2);
        if(inside_num != 0)
        {
            line_params = outside;
            measurements = outside_measurements;
        }
    }
    // Create new tracks based on the remaining measurements
    createTracks(measurements, line_params, initial_existence_probability);
}

void TrackManager::createTracks(Eigen::Array<double, 2, Eigen::Dynamic> measurements,
    Eigen::Array<double, 2, Eigen::Dynamic> line_params,
    double initial_existence_probability)
{
    for (Eigen::Index i = 0; i < line_params.cols(); ++i)
    {
        Eigen::Vector2d state_estimate;
        state_estimate << line_params(0, i), line_params(1, i);
        Track track;
        track.id = tracker_id_;
        track.state = vortex::prob::Gauss2d(state_estimate, Eigen::Matrix2d::Identity());
        track.line_points = measurements.block<2, 2>(0, i*2);
        track.existence_probability = initial_existence_probability;
        track.confirmed = false;
        tracks_.push_back(track);
        tracker_id_++;
    }
}

void TrackManager::deleteTracks(double deletion_threshold)
{
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [deletion_threshold](const Track &track) { return track.existence_probability < deletion_threshold; }), tracks_.end());
}

void TrackManager::set_dyn_model(double std_velocity)
{
    dyn_model_ = std::make_shared<DynMod>(std_velocity);
}

void TrackManager::set_sensor_model(double std_measurement)
{
    sensor_model_ = std::make_shared<SensorMod>(std_measurement);
}