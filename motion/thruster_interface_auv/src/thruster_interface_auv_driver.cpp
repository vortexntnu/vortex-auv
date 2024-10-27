#include "thruster_interface_auv/thruster_interface_auv_driver.hpp"

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver() {}

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver(
    short I2C_BUS,
    int PICO_I2C_ADDRESS,
    const std::vector<short>& THRUSTER_MAPPING,
    const std::vector<short>& THRUSTER_DIRECTION,
    const std::vector<int>& PWM_MIN,
    const std::vector<int>& PWM_MAX,
    const std::vector<double>& LEFT_COEFFS,
    const std::vector<double>& RIGHT_COEFFS)
    : I2C_BUS(I2C_BUS),
      PICO_I2C_ADDRESS(PICO_I2C_ADDRESS),
      THRUSTER_MAPPING(THRUSTER_MAPPING),
      THRUSTER_DIRECTION(THRUSTER_DIRECTION),
      PWM_MIN(PWM_MIN),
      PWM_MAX(PWM_MAX),
      LEFT_COEFFS(LEFT_COEFFS),
      RIGHT_COEFFS(RIGHT_COEFFS) {
    printf("I2C_BUS: %d\n", I2C_BUS);
    printf("PICO_I2C_ADDRESS: %d\n", PICO_I2C_ADDRESS);
    printf("THRUSTER_MAPPING: ");
    for (int i = 0; i < THRUSTER_MAPPING.size(); i++) {
        printf("%d ", THRUSTER_MAPPING[i]);
    }
    printf("\n");
    printf("THRUSTER_DIRECTION: ");
    for (int i = 0; i < THRUSTER_DIRECTION.size(); i++) {
        printf("%d ", THRUSTER_DIRECTION[i]);
    }
    printf("\n");
    printf("PWM_MIN: ");
    for (int i = 0; i < PWM_MIN.size(); i++) {
        printf("%d ", PWM_MIN[i]);
    }
    printf("\n");
    printf("PWM_MAX: ");
    for (int i = 0; i < PWM_MAX.size(); i++) {
        printf("%d ", PWM_MAX[i]);
    }
    printf("\n");
    printf("LEFT_COEFFS: ");
    for (int i = 0; i < LEFT_COEFFS.size(); i++) {
        printf("%f ", LEFT_COEFFS[i]);
    }
    printf("\n");
    printf("RIGHT_COEFFS: ");
    for (int i = 0; i < RIGHT_COEFFS.size(); i++) {
        printf("%f ", RIGHT_COEFFS[i]);
    }
    printf("\n");

    // Open the I2C bus
    std::string i2c_filename = "/dev/i2c-" + std::to_string(I2C_BUS);
    bus_fd = open(i2c_filename.c_str(),
                  O_RDWR);  // Open the i2c bus for reading and writing (0_RDWR)
    if (bus_fd < 0) {
        std::cerr << "ERROR: Failed to open I2C bus " << I2C_BUS << std::endl;
    }
}

ThrusterInterfaceAUVDriver::~ThrusterInterfaceAUVDriver() {
    if (bus_fd >= 0) {
        close(bus_fd);
    }
}

std::vector<int16_t> ThrusterInterfaceAUVDriver::interpolate_forces_to_pwm(
    const std::vector<double>& thruster_forces_array) {
    // Convert Newtons to Kg (since the thruster datasheet is in Kg)
    std::vector<double> forces_in_kg(thruster_forces_array.size());
    for (size_t i = 0; i < thruster_forces_array.size(); ++i) {
        forces_in_kg[i] = thruster_forces_array[i] / 9.80665;
    }

    std::vector<int16_t> interpolated_pwm;
    for (size_t i = 0; i < forces_in_kg.size(); ++i) {
        double force = forces_in_kg[i];
        double pwm = 0.0;
        if (force < 0) {
            pwm = LEFT_COEFFS[0] * std::pow(forces_in_kg[i], 3) +
                  LEFT_COEFFS[1] * std::pow(forces_in_kg[i], 2) +
                  LEFT_COEFFS[2] * forces_in_kg[i] + LEFT_COEFFS[3];
        } else if (force == 0.0) {
            pwm = 1500;
        } else {
            pwm = RIGHT_COEFFS[0] * std::pow(forces_in_kg[i], 3) +
                  RIGHT_COEFFS[1] * std::pow(forces_in_kg[i], 2) +
                  RIGHT_COEFFS[2] * forces_in_kg[i] + RIGHT_COEFFS[3];
        }
        interpolated_pwm.push_back(static_cast<int16_t>(pwm));
    }

    return interpolated_pwm;
}

void ThrusterInterfaceAUVDriver::send_data_to_escs(
    const std::vector<int16_t>& thruster_pwm_array) {
    uint8_t i2c_data_array[16];
    for (size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        i2c_data_array[2 * i] = (thruster_pwm_array[i] >> 8) & 0xFF;  // MSB
        i2c_data_array[2 * i + 1] = thruster_pwm_array[i] & 0xFF;     // LSB
    }

    // Set the I2C slave address
    if (ioctl(bus_fd, I2C_SLAVE, PICO_I2C_ADDRESS) < 0) {
        std::cerr << "ERROR: Failed to set I2C slave address" << std::endl;
        return;
    }

    // Write data to the I2C device
    if (write(bus_fd, i2c_data_array, 16) != 16) {
        std::cerr << "ERROR: Failed to write to I2C device" << std::endl;
    }
}

std::vector<int16_t> ThrusterInterfaceAUVDriver::drive_thrusters(
    const std::vector<double>& thruster_forces_array) {
    // Apply thruster mapping and direction
    std::vector<double> mapped_forces(thruster_forces_array.size());
    for (size_t i = 0; i < THRUSTER_MAPPING.size(); ++i) {
        mapped_forces[i] =
            thruster_forces_array[THRUSTER_MAPPING[i]] * THRUSTER_DIRECTION[i];
    }

    // Convert forces to PWM
    std::vector<int16_t> thruster_pwm_array =
        interpolate_forces_to_pwm(mapped_forces);

    // Apply thruster offset and limit PWM if needed
    for (size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        // Clamp the PWM signal
        if (thruster_pwm_array[i] < PWM_MIN[i]) {
            thruster_pwm_array[i] = PWM_MIN[i];
        } else if (thruster_pwm_array[i] > PWM_MAX[i]) {
            thruster_pwm_array[i] = PWM_MAX[i];
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
