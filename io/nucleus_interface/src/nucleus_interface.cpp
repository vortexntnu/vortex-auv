#include <vortex/io/nucleus_interface/nucleus_interface.hpp>

#include <cmath>
#include <numbers>
#include <type_traits>
#include <utility>
#include <variant>

#include <boost/system/error_code.hpp>

namespace vortex::io {
namespace driver = vortex::drivers::dvl;

NucleusInterface::NucleusInterface(boost::asio::io_context& io,
                                   NucleusInterfaceConfig config)
    : config_(std::move(config)),
      driver_(std::make_unique<driver::NortekNucleusDriver>(
          io,
          [this](driver::NortekNucleusFrame frame) {
              nucleus_callback(std::move(frame));
          })) {}

NucleusInterface::~NucleusInterface() {
    stop();
}

bool NucleusInterface::start() {
    if (!driver_) {
        return false;
    }

    driver::NortekConnectionParams connection_params{};
    connection_params.remote_ip = config_.remote_ip;
    connection_params.data_remote_port = config_.data_remote_port;
    connection_params.password = config_.password;

    const boost::system::error_code open_error =
        driver_->open_tcp_sockets(connection_params);

    if (open_error) {
        return false;
    }

    if (!connection_params.password.empty()) {
        const boost::system::error_code password_error =
            driver_->enter_password(connection_params);

        if (password_error) {
            return false;
        }
    }

    driver::InstrumentSettings instrument_settings{};
    instrument_settings.rotxy = config_.rotxy;
    instrument_settings.rotyz = config_.rotyz;
    instrument_settings.rotxz = config_.rotxz;

    if (driver_->set_instrument_settings(instrument_settings) !=
        driver::NucleusStatusCode::Ok) {
        return false;
    }

    if (config_.enable_imu) {
        driver::ImuSettings imu_settings{};
        imu_settings.freq = config_.imu_frequency_hz;
        imu_settings.data_stream_settings =
            driver::NucleusDataStreamSettings::On;
        imu_settings.data_format = driver::DataSeriesId::ImuData;

        if (driver_->set_imu_settings(imu_settings) !=
            driver::NucleusStatusCode::Ok) {
            return false;
        }

        driver::AhrsSettings ahrs_settings{};
        ahrs_settings.freq = config_.ahrs_frequency_hz;
        ahrs_settings.mode = config_.ahrs_mode;
        ahrs_settings.data_stream_settings =
            driver::NucleusDataStreamSettings::On;
        ahrs_settings.data_format = driver::DataSeriesId::AhrsData;

        if (driver_->set_ahrs_settings(ahrs_settings) !=
            driver::NucleusStatusCode::Ok) {
            return false;
        }
    }

    if (config_.enable_dvl) {
        driver::BottomTrackSettings bottom_track_settings{};
        bottom_track_settings.mode = config_.bottom_track_mode;
        bottom_track_settings.velocity_range =
            config_.bottom_track_velocity_range;
        bottom_track_settings.enable_watertrack = config_.enable_watertrack;
        bottom_track_settings.power_level_user_defined = false;
        bottom_track_settings.power_level = 0;
        bottom_track_settings.data_stream_settings =
            driver::NucleusDataStreamSettings::On;
        bottom_track_settings.data_format =
            driver::NucleusDataFormats::BottomTrackBinaryFormat;

        if (driver_->set_bottom_track_settings(bottom_track_settings) !=
            driver::NucleusStatusCode::Ok) {
            return false;
        }
    }

    if (config_.enable_altimeter || config_.enable_pressure) {
        driver::AltimeterSettings altimeter_settings{};
        altimeter_settings.power_level = config_.altimeter_power_level;
        altimeter_settings.data_stream_settings =
            driver::NucleusDataStreamSettings::On;
        altimeter_settings.data_format =
            driver::NucleusDataFormats::AltimeterFormat;

        if (driver_->set_altimeter_settings(altimeter_settings) !=
            driver::NucleusStatusCode::Ok) {
            return false;
        }
    }

    if (driver_->start_nucleus() != driver::NucleusStatusCode::Ok) {
        return false;
    }

    driver_->start_read();

    return true;
}

void NucleusInterface::stop() {
    if (driver_) {
        driver_->stop_nucleus();
    }
}

std::optional<NucleusState> NucleusInterface::latest_state() const {
    std::scoped_lock lock(state_mutex_);

    if (!latest_state_.valid) {
        return std::nullopt;
    }

    return latest_state_;
}

void NucleusInterface::nucleus_callback(driver::NortekNucleusFrame frame) {
    std::visit(
        [this](auto&& data) {
            using T = std::decay_t<decltype(data)>;

            if constexpr (std::is_same_v<T, driver::AhrsDataV2>) {
                handle_ahrs(data);
            } else if constexpr (std::is_same_v<T, driver::InsDataV2>) {
                handle_ins(data);
            } else if constexpr (std::is_same_v<T, driver::BottomTrackData>) {
                handle_bottom_track(data);
            } else if constexpr (std::is_same_v<T, driver::AltimeterData>) {
                handle_altimeter(data);
            }
        },
        std::move(frame));
}

void NucleusInterface::handle_ahrs(const driver::AhrsDataV2& data) {
    std::scoped_lock lock(state_mutex_);

    latest_qw_ = data.data_quaternion_w;
    latest_qx_ = data.data_quaternion_x;
    latest_qy_ = data.data_quaternion_y;
    latest_qz_ = data.data_quaternion_z;

    have_orientation_ = true;
}

void NucleusInterface::handle_ins(const driver::InsDataV2& data) {
    std::scoped_lock lock(state_mutex_);

    if (!have_orientation_) {
        return;
    }

    NucleusState state{};

    state.timestamp = std::chrono::steady_clock::now();

    // Nucleus position is NED. Keep this convention explicit unless your
    // vehicle-level Pose type intentionally converts to another frame.
    state.pose.x = data.position_ned_x;
    state.pose.y = data.position_ned_y;
    state.pose.z = data.position_ned_z;

    state.pose.qw = latest_qw_;
    state.pose.qx = latest_qx_;
    state.pose.qy = latest_qy_;
    state.pose.qz = latest_qz_;

    // Nucleus INS velocity is body-frame velocity.
    state.twist.u = data.velocity_body_x;
    state.twist.v = data.velocity_body_y;
    state.twist.w = data.velocity_body_z;

    constexpr double degrees_to_radians = std::numbers::pi_v<double> / 180.0;

    state.twist.p = data.turn_rate_x * degrees_to_radians;
    state.twist.q = data.turn_rate_y * degrees_to_radians;
    state.twist.r = data.turn_rate_z * degrees_to_radians;

    // Preserve values that arrive independently in altimeter packets.
    state.altitude_m = latest_state_.altitude_m;
    state.pressure_pa = latest_state_.pressure_pa;

    state.valid = true;
    latest_state_ = state;
}

void NucleusInterface::handle_bottom_track(
    const driver::BottomTrackData& data) {
    std::scoped_lock lock(state_mutex_);

    latest_dvl_.timestamp = std::chrono::steady_clock::now();

    latest_dvl_.velocity.x() = data.velocity_x;
    latest_dvl_.velocity.y() = data.velocity_y;
    latest_dvl_.velocity.z() = data.velocity_z;

    latest_dvl_.variance.x() = static_cast<double>(data.uncertainty_x) *
                               static_cast<double>(data.uncertainty_x);

    latest_dvl_.variance.y() = static_cast<double>(data.uncertainty_y) *
                               static_cast<double>(data.uncertainty_y);

    latest_dvl_.variance.z() = static_cast<double>(data.uncertainty_z) *
                               static_cast<double>(data.uncertainty_z);

    latest_dvl_.valid = true;
}

void NucleusInterface::handle_altimeter(const driver::AltimeterData& data) {
    std::scoped_lock lock(state_mutex_);

    latest_state_.altitude_m = static_cast<double>(data.distance);

    // Nucleus pressure is supplied in dBar.
    latest_state_.pressure_pa = static_cast<double>(data.pressure) * 10'000.0;
}

}  // namespace vortex::io
