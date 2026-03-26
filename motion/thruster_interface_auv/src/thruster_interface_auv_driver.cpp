#include "thruster_interface_auv/thruster_interface_auv_driver.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver(
    const std::string& serial_device,
    unsigned int baud_rate,
    std::uint8_t packet_id,
    const std::vector<ThrusterParameters>& thruster_parameters,
    const std::vector<double>& right_coeffs,
    const std::vector<double>& left_coeffs)
    : serial_device_(serial_device),
      baud_rate_(baud_rate),
      packet_id_(packet_id),
      thruster_parameters_(thruster_parameters),
      right_coeffs_(right_coeffs),
      left_coeffs_(left_coeffs) {
    idle_pwm_value_ =
        static_cast<std::uint16_t>(
            (calc_poly(0.0, left_coeffs_) + calc_poly(0.0, right_coeffs_)) / 2);
}

int ThrusterInterfaceAUVDriver::init_uart() {
    std::error_code ec;

    serial_.open(serial_device_, ec);
    if (ec) {
        std::cerr << "Failed to open serial port " << serial_device_
                  << ": " << ec.message() << '\n';
        return -1;
    }

    serial_.set_option(asio::serial_port_base::baud_rate(baud_rate_), ec);
    if (ec) return -1;

    serial_.set_option(asio::serial_port_base::character_size(8), ec);
    if (ec) return -1;

    serial_.set_option(
        asio::serial_port_base::parity(asio::serial_port_base::parity::none), ec);
    if (ec) return -1;

    serial_.set_option(
        asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one), ec);
    if (ec) return -1;

    serial_.set_option(
        asio::serial_port_base::flow_control(
            asio::serial_port_base::flow_control::none),
        ec);
    if (ec) return -1;

    return 0;
}

ThrusterInterfaceAUVDriver::~ThrusterInterfaceAUVDriver() {
    if (serial_.is_open()) {
        send_data_to_escs(
            std::vector<uint16_t>(thruster_parameters_.size(), idle_pwm_value_));

        std::error_code ec;
        serial_.close(ec);
    }
}

std::vector<uint16_t> ThrusterInterfaceAUVDriver::interpolate_forces_to_pwm(
    const std::vector<double>& thruster_forces_array) {
    std::vector<uint16_t> pwm(thruster_forces_array.size());

    for (std::size_t i = 0; i < thruster_forces_array.size(); ++i) {
        const double force_in_kg = to_kg(thruster_forces_array[i]);
        pwm[i] = force_to_pwm(force_in_kg);
    }

    return pwm;
}

std::uint16_t ThrusterInterfaceAUVDriver::force_to_pwm(double force) {
    if (force < 0.0) {
        return calc_poly(force, left_coeffs_);
    }
    if (force > 0.0) {
        return calc_poly(force, right_coeffs_);
    }
    return idle_pwm_value_;
}

std::uint16_t ThrusterInterfaceAUVDriver::calc_poly(
    double force,
    const std::vector<double>& coeffs) {
    return static_cast<std::uint16_t>(
        coeffs[0] * std::pow(force, 3) +
        coeffs[1] * std::pow(force, 2) +
        coeffs[2] * force +
        coeffs[3]);
}

std::vector<std::uint8_t> ThrusterInterfaceAUVDriver::create_packet(
    std::uint8_t id,
    const std::vector<uint16_t>& thruster_pwm_array) const {
    constexpr std::uint8_t magic = 0xAA;
    constexpr std::size_t expected_thrusters = 8;

    if (thruster_pwm_array.size() != expected_thrusters) {
        return {};
    }

    std::vector<std::uint8_t> packet;
    packet.reserve(1 + 1 + 1 + expected_thrusters * 2 + 1);

    packet.push_back(magic);
    packet.push_back(id);

    const std::uint8_t payload_length =
        static_cast<std::uint8_t>(thruster_pwm_array.size() * sizeof(std::uint16_t));
    packet.push_back(payload_length);

    for (std::uint16_t value : thruster_pwm_array) {
        // little-endian
        packet.push_back(static_cast<std::uint8_t>(value & 0xFF));
        packet.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    }

    std::uint8_t checksum = 0;
    for (std::uint8_t byte : packet) {
        checksum = static_cast<std::uint8_t>(checksum + byte);
    }

    packet.push_back(checksum);
    return packet;
}

int ThrusterInterfaceAUVDriver::send_data_to_escs(
    const std::vector<uint16_t>& thruster_pwm_array) {
    if (!serial_.is_open()) {
        return -1;
    }

    const auto packet = create_packet(packet_id_, thruster_pwm_array);
    if (packet.empty()) {
        return -1;
    }

    std::error_code ec;
    const auto bytes_written = asio::write(serial_, asio::buffer(packet), ec);

    if (ec || bytes_written != packet.size()) {
        std::cerr << "UART write failed: "
                  << (ec ? ec.message() : "short write") << '\n';
        return -1;
    }

    return 0;
}

std::optional<std::vector<uint16_t>> ThrusterInterfaceAUVDriver::drive_thrusters(
    const std::vector<double>& thruster_forces_array) {
    std::vector<double> mapped_forces(thruster_parameters_.size());

    for (std::size_t i = 0; i < thruster_parameters_.size(); ++i) {
        const auto& param = thruster_parameters_[i];
        const std::size_t idx = param.mapping;
        const double raw_force = thruster_forces_array[idx];
        mapped_forces[i] = raw_force * param.direction;
    }

    std::vector<uint16_t> thruster_pwm_array =
        interpolate_forces_to_pwm(mapped_forces);

    if (send_data_to_escs(thruster_pwm_array) != 0) {
        return std::nullopt;
    }

    return thruster_pwm_array;
}
