#include "thruster_interface_auv/thruster_interface_auv_driver.hpp"

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver(
    short i2c_bus,
    int pico_i2c_address,
    const std::vector<short>& thruster_mapping,
    const std::vector<short>& thruster_direction,
    const std::vector<int>& pwm_min,
    const std::vector<int>& pwm_max,
    const std::vector<double>& left_coeffs,
    const std::vector<double>& right_coeffs)
    : i2c_bus_(i2c_bus),
      pico_i2c_address_(pico_i2c_address),
      thruster_mapping_(thruster_mapping),
      thruster_direction_(thruster_direction),
      pwm_min_(pwm_min),
      pwm_max_(pwm_max),
      left_coeffs_(left_coeffs),
      right_coeffs_(right_coeffs) {
    printf("I2C_BUS: %d\n", i2c_bus_);
    printf("PICO_I2C_ADDRESS: %d\n", pico_i2c_address_);
    printf("THRUSTER_MAPPING: ");
    for (int i = 0; i < thruster_mapping_.size(); i++) {
        printf("%d ", thruster_mapping_[i]);
    }
    printf("\n");
    printf("THRUSTER_DIRECTION: ");
    for (int i = 0; i < thruster_direction_.size(); i++) {
        printf("%d ", thruster_direction_[i]);
    }
    printf("\n");
    printf("PWM_MIN: ");
    for (int i = 0; i < pwm_min_.size(); i++) {
        printf("%d ", pwm_min_[i]);
    }
    printf("\n");
    printf("PWM_MAX: ");
    for (int i = 0; i < pwm_max_.size(); i++) {
        printf("%d ", pwm_max_[i]);
    }
    printf("\n");
    printf("LEFT_COEFFS: ");
    for (int i = 0; i < left_coeffs_.size(); i++) {
        printf("%f ", left_coeffs_[i]);
    }
    printf("\n");
    printf("RIGHT_COEFFS: ");
    for (int i = 0; i < right_coeffs_.size(); i++) {
        printf("%f ", right_coeffs_[i]);
    }
    printf("\n");

    // Open the I2C bus
    std::string i2c_filename = "/dev/i2c-" + std::to_string(i2c_bus_);
    bus_fd_ =
        open(i2c_filename.c_str(),
             O_RDWR);  // Open the i2c bus for reading and writing (0_RDWR)
    if (bus_fd_ < 0) {
        std::cerr << "ERROR: Failed to open I2C bus " << i2c_bus_ << std::endl;
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
        interpolated_pwm.push_back(
            force_to_pwm(force, left_coeffs_, right_coeffs_));
    }
    return interpolated_pwm;
}

std::int16_t ThrusterInterfaceAUVDriver::force_to_pwm(
    double force,
    const std::vector<double>& left_coeffs,
    const std::vector<double>& right_coeffs) {
    if (force < 0) {
        return interpolate_pwm(force, left_coeffs);
    } else if (force > 0) {
        return interpolate_pwm(force, right_coeffs);
    } else {
        return 1500;
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
    uint8_t i2c_data_array[16];
    for (size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        i2c_data_array[2 * i] = (thruster_pwm_array[i] >> 8) & 0xFF;  // MSB
        i2c_data_array[2 * i + 1] = thruster_pwm_array[i] & 0xFF;     // LSB
    }

    // Set the I2C slave address
    if (ioctl(bus_fd_, I2C_SLAVE, pico_i2c_address_) < 0) {
        std::cerr << "ERROR: Failed to set I2C slave address" << std::endl;
        return;
    }

    // Write data to the I2C device
    if (write(bus_fd_, i2c_data_array, 16) != 16) {
        std::cerr << "ERROR: Failed to write to I2C device" << std::endl;
    }
}

std::vector<int16_t> ThrusterInterfaceAUVDriver::drive_thrusters(
    const std::vector<double>& thruster_forces_array) {
    // Apply thruster mapping and direction
    std::vector<double> mapped_forces(thruster_forces_array.size());
    for (size_t i = 0; i < thruster_mapping_.size(); ++i) {
        mapped_forces[i] = thruster_forces_array[thruster_mapping_[i]] *
                           thruster_direction_[i];
    }

    // Convert forces to PWM
    std::vector<int16_t> thruster_pwm_array =
        interpolate_forces_to_pwm(mapped_forces);

    // Apply thruster offset and limit PWM if needed
    for (size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        // Clamp the PWM signal
        if (thruster_pwm_array[i] < pwm_min_[i]) {
            thruster_pwm_array[i] = pwm_min_[i];
        } else if (thruster_pwm_array[i] > pwm_max_[i]) {
            thruster_pwm_array[i] = pwm_max_[i];
        }
    }

    // Send data through I2C
    try {
        send_data_to_escs(thruster_pwm_array);
    } catch (...) {
        std::cerr << "ERROR: Failed to send PWM values" << std::endl;
    }

    return thruster_pwm_array;
}
