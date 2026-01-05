#include "thruster_interface_auv/thruster_interface_auv_driver.hpp"
#include <spdlog/spdlog.h>
#include <cstdint>
#include <format>
#include <ranges>

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver(
    std::int16_t i2c_bus,
    int pico_i2c_address,
    const std::vector<ThrusterParameters>& thruster_parameters,
    const std::vector<std::vector<double>>& poly_coeffs)
    : i2c_bus_(i2c_bus),
      pico_i2c_address_(pico_i2c_address),
      thruster_parameters_(thruster_parameters),
      poly_coeffs_(poly_coeffs) {
    std::string i2c_filename = std::format("/dev/i2c-{}", i2c_bus_);
    bus_fd_ =
        open(i2c_filename.c_str(),
             O_RDWR);  // Open the i2c bus for reading and writing (0_RDWR)
    if (bus_fd_ < 0) {
        std::runtime_error(std::format("ERROR: Failed to open I2C bus {} : {}",
                                       i2c_bus_, strerror(errno)));
    }

    idle_pwm_value_ =
        (calc_poly(0, poly_coeffs_[LEFT]) + calc_poly(0, poly_coeffs_[RIGHT])) /
        2;
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
    // Convert Newtons to Kg (since the thruster datasheet is in Kg)
    auto pwm_view = thruster_forces_array | std::views::transform(to_kg) |
                    std::views::transform([this](double force_in_kg) {
                        return force_to_pwm(force_in_kg, poly_coeffs_);
                    });
    return std::vector<uint16_t>(pwm_view.begin(), pwm_view.end());
}

std::uint16_t ThrusterInterfaceAUVDriver::force_to_pwm(
    double force,
    const std::vector<std::vector<double>>& coeffs) {
    if (force < 0) {
        return calc_poly(force, coeffs[LEFT]);
    } else if (force > 0) {
        return calc_poly(force, coeffs[RIGHT]);
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

void ThrusterInterfaceAUVDriver::send_data_to_escs(
    const std::vector<uint16_t>& thruster_pwm_array) {
    constexpr std::size_t i2c_data_size =
        1 + 8 * 2;  // 8 thrusters * (1xMSB + 1xLSB)
    std::vector<std::uint8_t> i2c_data_array;
    i2c_data_array.reserve(i2c_data_size);

    i2c_data_array.push_back(0x00);  // Start byte
    std::ranges::for_each(thruster_pwm_array, [&](std::uint16_t pwm) {
        std::array<std::uint8_t, 2> bytes = pwm_to_i2c_data(pwm);
        std::ranges::copy(bytes, std::back_inserter(i2c_data_array));
    });

    // Set the I2C slave address
    if (ioctl(bus_fd_, I2C_SLAVE, pico_i2c_address_) < 0) {
        throw std::runtime_error(std::format("Failed to open I2C bus {} : {}",
                                             i2c_bus_, strerror(errno)));
        return;
    }

    // Write data to the I2C device
    if (write(bus_fd_, i2c_data_array.data(), i2c_data_size) != i2c_data_size) {
        throw std::runtime_error(std::format(
            "ERROR: Failed to write to I2C device : {}", strerror(errno)));
    }
}

std::vector<uint16_t> ThrusterInterfaceAUVDriver::drive_thrusters(
    const std::vector<double>& thruster_forces_array) {
    // Apply thruster mapping and direction
    std::vector<double> mapped_forces(thruster_forces_array.size());

    std::ranges::transform(thruster_parameters_, mapped_forces.begin(),
                           [this, &thruster_forces_array](const auto& param) {
                               return thruster_forces_array[param.mapping] *
                                      param.direction;
                           });

    // Convert forces to PWM
    std::vector<uint16_t> thruster_pwm_array =
        interpolate_forces_to_pwm(mapped_forces);

    std::ranges::transform(thruster_pwm_array, thruster_pwm_array.begin(),
                           [this, i = 0](auto pwm) mutable {
                               auto result = std::clamp(
                                   pwm, thruster_parameters_[i].pwm_min,
                                   thruster_parameters_[i].pwm_max);
                               ++i;
                               return result;
                           });

    try {
        send_data_to_escs(thruster_pwm_array);
    } catch (const std::exception& e) {
        spdlog::error("ERROR: Failed to send PWM values - {}", e.what());
    } catch (...) {
        spdlog::error("ERROR: Failed to send PWM values - Unknown exception");
    }

    return thruster_pwm_array;
}
