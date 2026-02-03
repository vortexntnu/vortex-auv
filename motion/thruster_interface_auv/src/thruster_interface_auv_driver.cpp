#include "thruster_interface_auv/thruster_interface_auv_driver.hpp"
#include <cstdint>
#include <cstring>
#include <format>

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver(
    std::int16_t i2c_bus,
    int pico_i2c_address,
    const std::vector<ThrusterParameters>& thruster_parameters,
    const std::vector<double>& right_coeffs,
    const std::vector<double>& left_coeffs)
    : i2c_bus_(i2c_bus),
      pico_i2c_address_(pico_i2c_address),
      thruster_parameters_(thruster_parameters),
      right_coeffs_(right_coeffs),
      left_coeffs_(left_coeffs) {
    idle_pwm_value_ =
        (calc_poly(0, left_coeffs_) + calc_poly(0, right_coeffs_)) / 2;
}

int ThrusterInterfaceAUVDriver::init_i2c() {
    std::string i2c_filename = std::format("/dev/i2c-{}", i2c_bus_);
    bus_fd_ = open(i2c_filename.c_str(), O_RDWR);
    if (bus_fd_ < 0) {
        return bus_fd_;
    }

    if (ioctl(bus_fd_, I2C_SLAVE, pico_i2c_address_) < 0) {
        return -1;
    }
    return 0;
}

ThrusterInterfaceAUVDriver::~ThrusterInterfaceAUVDriver() {
    if (bus_fd_ >= 0) {
        send_data_to_escs(std::vector<uint16_t>(thruster_parameters_.size(),
                                                idle_pwm_value_));
        close(bus_fd_);
    }
}

std::vector<uint16_t> ThrusterInterfaceAUVDriver::interpolate_forces_to_pwm(
    const std::vector<double>& thruster_forces_array) {
    std::vector<uint16_t> pwm;
    pwm.resize(thruster_forces_array.size());

    for (std::size_t i = 0; i < thruster_forces_array.size(); ++i) {
        const double force_in_kg = to_kg(thruster_forces_array[i]);
        pwm[i] = force_to_pwm(force_in_kg);
    }

    return pwm;
}

std::uint16_t ThrusterInterfaceAUVDriver::force_to_pwm(double force) {
    if (force < 0) {
        return calc_poly(force, left_coeffs_);
    } else if (force > 0) {
        return calc_poly(force, right_coeffs_);
    } else {
        return idle_pwm_value_;  // 1500
    }
}

std::uint16_t ThrusterInterfaceAUVDriver::calc_poly(
    double force,
    const std::vector<double>& coeffs) {
    return static_cast<std::uint16_t>(coeffs[0] * std::pow(force, 3) +
                                      coeffs[1] * std::pow(force, 2) +
                                      coeffs[2] * force + coeffs[3]);
}

int ThrusterInterfaceAUVDriver::send_data_to_escs(
    const std::vector<uint16_t>& thruster_pwm_array) {
    constexpr std::size_t i2c_data_size = 8 * 2;
    std::array<uint8_t, i2c_data_array> i2c_data_array;

    std::memcpy(i2c_data_array.data(), thruster_pwm_array.data(),
                i2c_data_size);

    if (write(bus_fd_, i2c_data_array.data(), i2c_data_size) != i2c_data_size) {
        return -1;
    }

    return 0;
}

std::optional<std::vector<uint16_t>> ThrusterInterfaceAUVDriver::drive_thrusters(
    const std::vector<double>& thruster_forces_array) {
    std::vector<double> mapped_forces(thruster_forces_array.size());

    for (std::size_t i = 0; i < thruster_parameters_.size(); ++i) {
        const auto& param = thruster_parameters_[i];

        const std::size_t idx = param.mapping;

        const double raw_force = thruster_forces_array[idx];
        mapped_forces[i] = raw_force * param.direction;
    }

    std::vector<uint16_t> thruster_pwm_array =
        interpolate_forces_to_pwm(mapped_forces);

    if (send_data_to_escs(thruster_pwm_array)) {
        return {};
    }

    return thruster_pwm_array;
}
