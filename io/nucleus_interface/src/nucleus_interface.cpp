#include <vortex/io/nucleus_interface/nucleus_interface.hpp>

#include <cmath>
#include <utility>

namespace vortex::io {

NucleusInterface::NucleusInterface(const NucleusInterfaceConfig& config)
    : config_(config) {
    driver_ = std::make_unique<NortekNucleusDriver>(
        io_, [this](NortekNucleusFrame frame) {
            nucleus_callback(std::move(frame));
        });
}

NucleusInterface::~NucleusInterface() {
    stop();
}

bool NucleusInterface::start() {
    NortekConnectionParams connection_params{};
    connection_params.remote_ip = config_.remote_ip;
    connection_params.data_remote_port = config_.data_remote_port;
    connection_params.password = config_.password;

    const std::error_code open_error =
        driver_->open_tcp_sockets(connection_params);

    if (open_error) {
        return false;
    }

    if (!connection_params.password.empty()) {
        const std::error_code error =
            driver_->enter_password(connection_params);

        if (error) {
            return false;
        }
    }

    InstrumentSettings instrument_settings{};
    instrument_settings.rotxy = config_.rotxy;
    instrument_settings.rotyz = config_.rotyz;
    instrument_settings.rotxz = config_.rotxz;

    if (driver_->set_instrument_settings(instrument_settings) !=
        NucleusStatusCode::Ok) {
        return false;
    }

    if (config_.enable_imu) {
        ImuSettings imu_settings{};
        imu_settings.freq = config_.imu_frequency_hz;
        imu_settings.data_stream_settings = NucleusDataStreamSettings::On;
        imu_settings.data_format = DataSeriesId::ImuData;

        if (driver_->set_imu_settings(imu_settings) != NucleusStatusCode::Ok) {
            return false;
        }

        AhrsSettings ahrs_settings{};
        ahrs_settings.freq = config_.ahrs_frequency_hz;
        ahrs_settings.mode = config_.ahrs_mode;
        ahrs_settings.data_stream_settings = NucleusDataStreamSettings::On;
        ahrs_settings.data_format = DataSeriesId::AhrsData;

        if (driver_->set_ahrs_settings(ahrs_settings) !=
            NucleusStatusCode::Ok) {
            return false;
        }
    }

    if (config_.enable_dvl) {
        BottomTrackSettings bottom_track_settings{};
        bottom_track_settings.mode = config_.bottom_track_mode;
        bottom_track_settings.velocity_range =
            config_.bottom_track_velocity_range;
        bottom_track_settings.enable_watertrack = config_.enable_watertrack;
        bottom_track_settings.power_level_user_defined = false;
        bottom_track_settings.power_level = 0;
        bottom_track_settings.data_stream_settings =
            NucleusDataStreamSettings::On;
        bottom_track_settings.data_format =
            NucleusDataFormats::BottomTrackBinaryFormat;

        if (driver_->set_bottom_track_settings(bottom_track_settings) !=
            NucleusStatusCode::Ok) {
            return false;
        }
    }

    if (config_.enable_altimeter || config_.enable_pressure) {
        AltimeterSettings altimeter_settings{};
        altimeter_settings.power_level = config_.altimeter_power_level;
        altimeter_settings.data_stream_settings = NucleusDataStreamSettings::On;
        altimeter_settings.data_format = NucleusDataFormats::AltimeterFormat;

        if (driver_->set_altimeter_settings(altimeter_settings) !=
            NucleusStatusCode::Ok) {
            return false;
        }
    }

    driver_->start_read();

    if (driver_->start_nucleus() != NucleusStatusCode::Ok) {
        return false;
    }

    io_thread_ = std::jthread([this] { io_.run(); });

    return true;
}

void NucleusInterface::stop() {
    if (driver_) {
        // Add a driver_->stop_nucleus() call here if the driver has it.
        // Do not invent one unless it exists in the driver API.
    }

    io_.stop();

    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

std::optional<NucleusState> NucleusInterface::latest_state() const {
    std::scoped_lock lock(state_mutex_);

    if (!latest_state_.valid) {
        return std::nullopt;
    }

    return latest_state_;
}

void NucleusInterface::nucleus_callback(NortekNucleusFrame frame) {
    std::visit(
        [this](auto&& data) {
            using T = std::decay_t<decltype(data)>;

            if constexpr (std::is_same_v<T, AhrsDataV2>) {
                handle_ahrs(data);
            } else if constexpr (std::is_same_v<T, InsDataV2>) {
                handle_ins(data);
            } else if constexpr (std::is_same_v<T, BottomTrackData>) {
                handle_bottom_track(data);
            } else if constexpr (std::is_same_v<T, AltimeterData>) {
                handle_altimeter(data);
            }
        },
        frame);
}

void NucleusInterface::handle_ahrs(const AhrsDataV2& data) {
    std::scoped_lock lock(state_mutex_);

    latest_qw_ = data.data_quaternion_w;
    latest_qx_ = data.data_quaternion_x;
    latest_qy_ = data.data_quaternion_y;
    latest_qz_ = data.data_quaternion_z;

    have_orientation_ = true;
}
void NucleusInterface::handle_ins(const InsDataV2& data) {
    std::scoped_lock lock(state_mutex_);

    if (!have_orientation_) {
        return;
    }

    NucleusState state{};

    state.timestamp = std::chrono::steady_clock::now();

    // Keep this coordinate convention explicit:
    // Nucleus gives NED position data.
    state.pose.x = data.position_ned_x;
    state.pose.y = data.position_ned_y;
    state.pose.z = data.position_ned_z;

    // Replace these with the actual fields in your Pose type.
    //
    // If Pose stores quaternion:
    state.pose.qw = latest_qw_;
    state.pose.qx = latest_qx_;
    state.pose.qy = latest_qy_;
    state.pose.qz = latest_qz_;

    // Nucleus INS velocity is body-frame velocity.
    state.twist.u = data.velocity_body_x;
    state.twist.v = data.velocity_body_y;
    state.twist.w = data.velocity_body_z;

    // Original ROS interface converted these from degrees/s to rad/s.
    constexpr double degrees_to_radians = std::numbers::pi_v<double> / 180.0;

    state.twist.p = data.turn_rate_x * degrees_to_radians;
    state.twist.q = data.turn_rate_y * degrees_to_radians;
    state.twist.r = data.turn_rate_z * degrees_to_radians;

    // Preserve newest auxiliary values received from their own packets.
    state.altitude_m = latest_state_.altitude_m;
    state.pressure_pa = latest_state_.pressure_pa;

    state.valid = true;

    latest_state_ = state;
}

void NucleusInterface::handle_bottom_track(const BottomTrackData& data) {
    std::scoped_lock lock(state_mutex_);

    latest_dvl_.timestamp = std::chrono::steady_clock::now();

    latest_dvl_.velocity.x() = data.velocity_x;
    latest_dvl_.velocity.y() = data.velocity_y;
    latest_dvl_.velocity.z() = data.velocity_z;

    latest_dvl_.variance.x() =
        static_cast<double>(data.uncertainty_x * data.uncertainty_x);

    latest_dvl_.variance.y() =
        static_cast<double>(data.uncertainty_y * data.uncertainty_y);

    latest_dvl_.variance.z() =
        static_cast<double>(data.uncertainty_z * data.uncertainty_z);

    latest_dvl_.valid = true;
}

void NucleusInterface::handle_altimeter(const AltimeterData& data) {
    std::scoped_lock lock(state_mutex_);

    latest_state_.altitude_m = static_cast<double>(data.distance);

    // Nucleus gives pressure in dBar.
    latest_state_.pressure_pa = static_cast<double>(data.pressure) * 10'000.0;
}

}  // namespace vortex::io
