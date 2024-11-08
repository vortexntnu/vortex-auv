#include "thruster_interface_auv/thruster_interface_auv_driver.hpp"

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver(
    short i2c_bus,
    int pico_i2c_address,
    const std::vector<ThrusterParameters>& thruster_parameters,
    const std::vector<PolyCoeffs>& poly_coeffs)
    : i2c_bus_(i2c_bus),
      pico_i2c_address_(pico_i2c_address),
      thruster_parameters_(thruster_parameters),
      poly_coeffs_(poly_coeffs) {
    // TO BE REMOVED----------------------------------------------------
    printf("I2C_BUS: %d\n", i2c_bus_);
    printf("PICO_I2C_ADDRESS: %d\n", pico_i2c_address_);
    printf("THRUSTER_MAPPING: ");
    for (size_t i = 0; i < thruster_parameters_[0].mapping.size(); ++i) {
        printf("%d ", thruster_parameters_[0].mapping[i]);
    }
    printf("\n");
    printf("THRUSTER_DIRECTION: ");
    for (size_t i = 0; i < thruster_parameters_[0].direction.size(); ++i) {
        printf("%d ", thruster_parameters_[0].direction[i]);
    }
    printf("\n");
    printf("THRUSTER_PWM_MIN: ");
    for (size_t i = 0; i < thruster_parameters_[0].pwm_min.size(); ++i) {
        printf("%d ", thruster_parameters_[0].pwm_min[i]);
    }
    printf("\n");
    printf("THRUSTER_PWM_MAX: ");
    for (size_t i = 0; i < thruster_parameters_[0].pwm_max.size(); ++i) {
        printf("%d ", thruster_parameters_[0].pwm_max[i]);
    }
    printf("\n");
    printf("LEFT_COEFFS: ");
    for (size_t i = 0; i < poly_coeffs_[0].left.size(); ++i) {
        printf("%f ", poly_coeffs_[0].left[i]);
    }
    printf("\n");
    printf("RIGHT_COEFFS: ");
    for (size_t i = 0; i < poly_coeffs_[0].right.size(); ++i) {
        printf("%f ", poly_coeffs_[0].right[i]);
    }
    printf("\n");
    //----------------------------------------------------------------

    // Open the I2C bus
    std::string i2c_filename = "/dev/i2c-" + std::to_string(i2c_bus_);
    bus_fd_ =
        open(i2c_filename.c_str(),
             O_RDWR);  // Open the i2c bus for reading and writing (0_RDWR)
    if (bus_fd_ < 0) {
        std::runtime_error("ERROR: Failed to open I2C bus " +
                           std::to_string(i2c_bus_) + " : " +
                           std::string(strerror(errno)));
    }
}

ThrusterInterfaceAUVDriver::~ThrusterInterfaceAUVDriver() {
    if (bus_fd_ >= 0) {
        close(bus_fd_);
    }
}

std::vector<int16_t> ThrusterInterfaceAUVDriver::interpolate_forces_to_pwm(
    const std::vector<double>& thruster_forces_array) {
    // Convert Newtons to Kg (since the thruster datasheet is in Kg)
    std::vector<double> forces_in_kg(thruster_forces_array.size());
    std::transform(thruster_forces_array.begin(), thruster_forces_array.end(),
                   forces_in_kg.begin(), to_kg);

    std::vector<int16_t> interpolated_pwm;
    for (const double force : forces_in_kg) {
        interpolated_pwm.push_back(force_to_pwm(force, poly_coeffs_));
    }
    return interpolated_pwm;
}

std::int16_t ThrusterInterfaceAUVDriver::force_to_pwm(
    double force,
    const std::vector<PolyCoeffs>& coeffs) {
    if (force < 0) {
        return interpolate_pwm(force, coeffs[0].left);
    } else if (force > 0) {
        return interpolate_pwm(force, coeffs[0].right);
    } else {
        return IDLE_PWM_VALUE;  // 1500
    }
}

std::int16_t ThrusterInterfaceAUVDriver::interpolate_pwm(
    double force,
    const std::vector<double>& coeffs) {
    return static_cast<std::int16_t>(coeffs[0] * std::pow(force, 3) +
                                     coeffs[1] * std::pow(force, 2) +
                                     coeffs[2] * force + coeffs[3]);
}

void ThrusterInterfaceAUVDriver::send_data_to_escs(
    const std::vector<int16_t>& thruster_pwm_array) {
    constexpr std::size_t i2c_data_size =
        8 * 2;  // 8 thrusters * (1xMSB + 1xLSB)
    std::vector<std::uint8_t> i2c_data_array;
    i2c_data_array.reserve(i2c_data_size);

    std::for_each(thruster_pwm_array.begin(), thruster_pwm_array.end(),
                  [&](std::int16_t pwm) {
                      std::array<std::uint8_t, 2> bytes = pwm_to_i2c_data(pwm);
                      std::copy(bytes.begin(), bytes.end(),
                                std::back_inserter(i2c_data_array));
                  });

    // Set the I2C slave address
    if (ioctl(bus_fd_, I2C_SLAVE, pico_i2c_address_) < 0) {
        throw std::runtime_error("Failed to open I2C bus " +
                                 std::to_string(i2c_bus_) + " : " +
                                 std::string(strerror(errno)));
        return;
    }

    // Write data to the I2C device
    if (write(bus_fd_, i2c_data_array.data(), 16) != 16) {
        throw std::runtime_error("ERROR: Failed to write to I2C device : " +
                                 std::string(strerror(errno)));
    }
}

std::vector<int16_t> ThrusterInterfaceAUVDriver::drive_thrusters(
    const std::vector<double>& thruster_forces_array) {
    // Apply thruster mapping and direction
    std::vector<double> mapped_forces(thruster_forces_array.size());
    for (size_t i = 0; i < thruster_parameters_[0].mapping.size(); ++i) {
        mapped_forces[i] =
            thruster_forces_array[thruster_parameters_[0].mapping[i]] *
            thruster_parameters_[0].direction[i];
    }

    // Convert forces to PWM
    std::vector<int16_t> thruster_pwm_array =
        interpolate_forces_to_pwm(mapped_forces);

    // Apply thruster offset and limit PWM if needed
    for (size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        // Clamp the PWM signal
        if (thruster_pwm_array[i] < thruster_parameters_[0].pwm_min[i]) {
            thruster_pwm_array[i] = thruster_parameters_[0].pwm_min[i];
        } else if (thruster_pwm_array[i] > thruster_parameters_[0].pwm_max[i]) {
            thruster_pwm_array[i] = thruster_parameters_[0].pwm_max[i];
        }
    }

    try {
        send_data_to_escs(thruster_pwm_array);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to send PWM values - " << e.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "ERROR: Failed to send PWM values - Unknown exception"
                  << std::endl;
    }

    return thruster_pwm_array;
}
