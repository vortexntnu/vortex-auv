#include <line_filtering/track_manager.hpp>

TrackManager::TrackManager()
: tracker_id_(0)
{
}

void TrackManager::updateTracks(Eigen::Array<double, 2, Eigen::Dynamic> measurements_,
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
            measurements_, 
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
        Eigen::Array<double, 2, Eigen::Dynamic> outside(2, measurements_.cols());
    
        Eigen::Index inside_num = 0;
        for (Eigen::Index i = 0; i < measurements_.cols(); ++i)
        {
            if (output.gated_measurements[i])
            {
                track.cluster.insert(track.cluster.end(), clusters_[i].begin(), clusters_[i].end());

                z_centroid_meas += centroid_z_meas_[i];
                inside_num++;
            }
            else
            {
                clusters.push_back(clusters_[i]);
                outside.col(i-inside_num) = measurements_.col(i);
                centroid_z_meas.push_back(centroid_z_meas_[i]);
            }
        }
        outside.conservativeResize(2, measurements_.cols() - inside_num);
        if(inside_num != 0)
        {
            track.centroid_z_measurement = z_centroid_meas / inside_num;
            centroid_z_meas_ = centroid_z_meas;
            clusters_ = clusters;
            measurements_ = outside;
        }
    }
    // Create new tracks based on the remaining measurements
    createTracks(measurements_, initial_existence_probability);
}

void TrackManager::createTracks(Eigen::Array<double, 2, Eigen::Dynamic> measurements, 
    double initial_existence_probability)
{
        
    for (Eigen::Index i = 0; i < measurements.cols(); ++i)
    {
        Eigen::Vector4d state_estimate;
        state_estimate << measurements.col(i), 0.0, 0.0;
        Track track;
        track.id = tracker_id_;
        track.state = vortex::prob::Gauss4d(state_estimate, Eigen::Matrix4d::Identity());
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