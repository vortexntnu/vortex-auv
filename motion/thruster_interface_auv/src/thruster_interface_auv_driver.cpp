#include "thruster_interface_auv/thruster_interface_auv_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <thread>

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver(
    const std::string& serial_device,
    unsigned int baud_rate,
    const std::vector<ThrusterParameters>& thruster_parameters,
    const std::vector<double>& right_coeffs,
    const std::vector<double>& left_coeffs)
    : serial_device_(serial_device),
      baud_rate_(baud_rate),
      thruster_parameters_(thruster_parameters),
      right_coeffs_(right_coeffs),
      left_coeffs_(left_coeffs) {
    idle_pwm_value_ = static_cast<std::uint16_t>(
        (calc_poly(0.0, left_coeffs_) + calc_poly(0.0, right_coeffs_)) / 2);
}

ThrusterInterfaceAUVDriver::~ThrusterInterfaceAUVDriver() {
    std::error_code ec;

    if (serial_.is_open()) {
        send_data_to_escs(
            std::vector<std::uint16_t>(thruster_parameters_.size(), idle_pwm_value_));

        serial_.cancel(ec);
        serial_.close(ec);
    }

    io_.stop();

    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

int ThrusterInterfaceAUVDriver::init_uart() {
    std::error_code ec;

    serial_.open(serial_device_, ec);
    if (ec) {
        return -1;
    }

    serial_.set_option(asio::serial_port::baud_rate(baud_rate_), ec);
    if (ec) {
        return -1;
    }

    serial_.set_option(asio::serial_port::character_size(8), ec);
    if (ec) {
        return -1;
    }

    serial_.set_option(
        asio::serial_port::parity(asio::serial_port::parity::none), ec);
    if (ec) {
        return -1;
    }

    serial_.set_option(
        asio::serial_port::stop_bits(asio::serial_port::stop_bits::one), ec);
    if (ec) {
        return -1;
    }

    serial_.set_option(
        asio::serial_port::flow_control(asio::serial_port::flow_control::none),
        ec);
    if (ec) {
        return -1;
    }

    start_receive();
    io_thread_ = std::thread([this]() { io_.run(); });

    return 0;
}

std::vector<std::uint16_t> ThrusterInterfaceAUVDriver::interpolate_forces_to_pwm(
    const std::vector<double>& thruster_forces_array) {
    std::vector<std::uint16_t> pwm(thruster_forces_array.size());

    for (std::size_t i = 0; i < thruster_forces_array.size(); ++i) {
        const double force_in_kg = to_kg(thruster_forces_array[i]);
        pwm[i] = force_to_pwm(force_in_kg);
    }

    return pwm;
}

std::uint16_t ThrusterInterfaceAUVDriver::force_to_pwm(double force) {
    constexpr double deadband_kg = 0.03;

    if (std::abs(force) < deadband_kg) {
        return idle_pwm_value_;
    }

    if (force < 0.0) {
        return calc_poly(force, left_coeffs_);
    }

    return calc_poly(force, right_coeffs_);
}

std::uint16_t ThrusterInterfaceAUVDriver::calc_poly(
    double force,
    const std::vector<double>& coeffs) {
    return static_cast<std::uint16_t>(coeffs[0] * std::pow(force, 3) +
                                      coeffs[1] * std::pow(force, 2) +
                                      coeffs[2] * force + coeffs[3]);
}

std::vector<std::uint8_t> ThrusterInterfaceAUVDriver::create_packet(
    std::uint8_t id,
    const std::vector<std::uint16_t>& thruster_pwm_array) const {
    std::vector<std::uint8_t> packet;

    packet.reserve(1 + 1 + 1 +
                   thruster_pwm_array.size() * sizeof(std::uint16_t) + 1);

    packet.push_back(UART_START_BYTE);
    packet.push_back(id);

    const std::uint8_t length = static_cast<std::uint8_t>(
        thruster_pwm_array.size() * sizeof(std::uint16_t));
    packet.push_back(length);

    for (std::uint16_t value : thruster_pwm_array) {
        packet.push_back(static_cast<std::uint8_t>(value & 0xFF));
        packet.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    }

    std::uint8_t checksum = id ^ length;
    for (std::uint16_t value : thruster_pwm_array) {
        checksum ^= static_cast<std::uint8_t>(value & 0xFF);
        checksum ^= static_cast<std::uint8_t>((value >> 8) & 0xFF);
    }

    packet.push_back(checksum);

    return packet;
}

int ThrusterInterfaceAUVDriver::send_data_to_escs(
    const std::vector<std::uint16_t>& thruster_pwm_array) {
    if (!serial_.is_open()) {
        return -1;
    }

    const auto packet = create_packet(MSG_SET_THRUSTER_PWM, thruster_pwm_array);
    constexpr std::size_t header_size = 3;

    if (packet.size() < header_size) {
        return -1;
    }

    std::error_code ec;

    const auto header_bytes_written =
        asio::write(serial_, asio::buffer(packet.data(), header_size), ec);

    if (ec || header_bytes_written != header_size) {
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    const auto remaining_size = packet.size() - header_size;
    const auto payload_bytes_written =
        asio::write(serial_,
                    asio::buffer(packet.data() + header_size, remaining_size),
                    ec);

    if (ec || payload_bytes_written != remaining_size) {
        return -1;
    }

    return 0;
}

int ThrusterInterfaceAUVDriver::set_camera_light(float percentage) {
    if (!serial_.is_open()) {
        return -1;
    }

    std::vector<std::uint16_t> camera_light_pwm_array(1);
    camera_light_pwm_array[0] =
        static_cast<std::uint16_t>(1100 + 800 * percentage);

    const auto packet = create_packet(MSG_SET_LIGHT_PWM, camera_light_pwm_array);
    constexpr std::size_t header_size = 3;

    if (packet.size() < header_size) {
        return -1;
    }

    std::error_code ec;

    const auto header_bytes_written =
        asio::write(serial_, asio::buffer(packet.data(), header_size), ec);

    if (ec || header_bytes_written != header_size) {
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    const auto remaining_size = packet.size() - header_size;
    const auto payload_bytes_written =
        asio::write(serial_,
                    asio::buffer(packet.data() + header_size, remaining_size),
                    ec);

    if (ec || payload_bytes_written != remaining_size) {
        return -1;
    }

    return 0;
}

std::optional<std::vector<std::uint16_t>>
ThrusterInterfaceAUVDriver::drive_thrusters(
    const std::vector<double>& thruster_forces_array) {
    std::vector<double> mapped_forces(thruster_parameters_.size());

    for (std::size_t i = 0; i < thruster_parameters_.size(); ++i) {
        const auto& param = thruster_parameters_[i];
        const std::size_t idx = param.mapping;
        const double raw_force = thruster_forces_array[idx];
        mapped_forces[i] = raw_force * param.direction;
    }

    std::vector<std::uint16_t> thruster_pwm_array =
        interpolate_forces_to_pwm(mapped_forces);

    if (send_data_to_escs(thruster_pwm_array) != 0) {
        return std::nullopt;
    }

    return thruster_pwm_array;
}

void ThrusterInterfaceAUVDriver::start_receive() {
    do_receive();
}

void ThrusterInterfaceAUVDriver::do_receive() {
    serial_.async_read_some(
        asio::buffer(read_buf_),
        [this](const std::error_code& ec, std::size_t bytes_transferred) {
            if (ec) {
                return;
            }

            receive_buffer_.insert(receive_buffer_.end(), read_buf_.begin(),
                                   read_buf_.begin() + bytes_transferred);

            process_receive_buffer();
            do_receive();
        });
}

std::uint8_t ThrusterInterfaceAUVDriver::compute_checksum(
    std::uint8_t msg_id,
    std::uint8_t length,
    const std::uint8_t* payload) {
    std::uint8_t checksum = msg_id ^ length;

    for (std::size_t i = 0; i < length; ++i) {
        checksum ^= payload[i];
    }

    return checksum;
}

void ThrusterInterfaceAUVDriver::process_receive_buffer() {
    while (true) {
        if (receive_buffer_.size() < 4) {
            return;
        }

        auto start_it = std::find(receive_buffer_.begin(), receive_buffer_.end(),
                                  UART_START_BYTE);

        if (start_it == receive_buffer_.end()) {
            receive_buffer_.clear();
            return;
        }

        if (start_it != receive_buffer_.begin()) {
            receive_buffer_.erase(receive_buffer_.begin(), start_it);
        }

        if (receive_buffer_.size() < 4) {
            return;
        }

        const std::uint8_t start = receive_buffer_[0];
        const std::uint8_t msg_id = receive_buffer_[1];
        const std::uint8_t length = receive_buffer_[2];

        if (start != UART_START_BYTE) {
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }

        if (length > MAX_PAYLOAD_SIZE) {
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }

        const std::size_t full_frame_size =
            4u + static_cast<std::size_t>(length);

        if (receive_buffer_.size() < full_frame_size) {
            return;
        }

        const std::uint8_t* payload_ptr = receive_buffer_.data() + 3;
        const std::uint8_t received_checksum = receive_buffer_[3 + length];
        const std::uint8_t expected_checksum =
            compute_checksum(msg_id, length, payload_ptr);

        if (received_checksum != expected_checksum) {
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }

        std::vector<std::uint8_t> frame_bytes(
            receive_buffer_.begin(),
            receive_buffer_.begin() +
                static_cast<std::ptrdiff_t>(full_frame_size));

        handle_received_frame(frame_bytes);

        receive_buffer_.erase(
            receive_buffer_.begin(),
            receive_buffer_.begin() +
                static_cast<std::ptrdiff_t>(full_frame_size));
    }
}

void ThrusterInterfaceAUVDriver::handle_received_frame(
    const std::vector<std::uint8_t>& frame_bytes) {
    if (frame_bytes.size() < 4) {
        return;
    }

    const std::uint8_t msg_id = frame_bytes[1];
    const std::uint8_t length = frame_bytes[2];

    if (frame_bytes.size() != static_cast<std::size_t>(length) + 4u) {
        return;
    }

    const std::uint8_t* payload = frame_bytes.data() + 3;

    switch (msg_id) {
        case MSG_FLT_EVENT: {
            if (length != 2) {
                break;
            }

            const std::uint8_t channel = payload[0];
            const std::uint8_t code = payload[1];

            if (fault_event_callback_) {
                fault_event_callback_(channel, code);
            }
            break;
        }

        case MSG_PGOOD_EVENT: {
            if (length != 2) {
                break;
            }

            const std::uint8_t channel = payload[0];
            const std::uint8_t code = payload[1];

            if (pgood_event_callback_) {
                pgood_event_callback_(channel, code);
            }
            break;
        }

        case MSG_KILLSWITCH_EVENT: {
            if (length != 0) {
                break;
            }

            if (killswitch_event_callback_) {
                killswitch_event_callback_();
            }
            break;
        }

        case MSG_CURRENT_MEASUREMENTS: {
            constexpr std::size_t num_currents = 8;
            constexpr std::size_t expected_length = num_currents * sizeof(float);

            if (length != expected_length) {
                break;
            }

            std::array<float, num_currents> currents{};

            std::memcpy(currents.data(), payload, expected_length);

            if (current_measurements_callback_) {
                current_measurements_callback_(currents);
            }
            break;
        }

        default: {
            break;
        }
    }
}

void ThrusterInterfaceAUVDriver::set_fault_event_callback(
    FaultEventCallback callback) {
    fault_event_callback_ = std::move(callback);
}

void ThrusterInterfaceAUVDriver::set_pgood_event_callback(
    PGoodEventCallback callback) {
    pgood_event_callback_ = std::move(callback);
}

void ThrusterInterfaceAUVDriver::set_killswitch_event_callback(
    KillswitchEventCallback callback) {
    killswitch_event_callback_ = std::move(callback);
}

void ThrusterInterfaceAUVDriver::set_current_measurements_callback(
    CurrentMeasurementsCallback callback) {
    current_measurements_callback_ = std::move(callback);
}
