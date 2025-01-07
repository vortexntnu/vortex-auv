#ifndef THRUSTER_INTERFACE_AUV_DRIVER_HPP
#define THRUSTER_INTERFACE_AUV_DRIVER_HPP

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

/**
 * @brief struct to hold the parameters for a single thruster
 */
struct ThrusterParameters {
    uint8_t mapping;
    int8_t direction;
    uint16_t pwm_min;
    uint16_t pwm_max;
};

enum PolySide {
    LEFT = 0,
    RIGHT = 1
};  // vector index for the position of the coefficients in the coeff vector

/**
 * @brief class instantiated by ThrusterInterfaceAUVNode to control the
 * thrusters, takes the thruster forces and converts them to PWM signals to be
 * sent via I2C to the ESCs (PCA9685 Adafruit 16-Channel 12-bit PWM/Servo
 * Driver)
 *
 * @details Based on the datasheets found in /resources, approximate the map
 * with a piecewise (>0 and <0) third order polynomial.
 *
 * @note The coefficients are found in the config.yaml for all the possible
 * SYSTEM.OPERATIONAL_VOLTAGE values, but we use only 16V for now, so: removed
 * all the handling of the other voltages to save resources. Could be
 * re-implemented in the future for more flexibility if we ever need it to
 * operate at different voltages in different situations.
 */
class ThrusterInterfaceAUVDriver {
   public:
    ~ThrusterInterfaceAUVDriver();

    /**
     * @brief called from ThrusterInterfaceAUVNode .cpp when instantiating the
     * object, initializes all the params.
     *
     * @param i2c_bus               bus number used to communicate
     * @param pico_i2c_address      i2c address of the ESC that drive the
     * @param thruster_parameters   describe mapping, direction, min and max pwm
     * value for each thruster
     * @param poly_coeffs           LEFT(<0) and RIGHT(>0) third order
     * polynomial coefficients
     */
    ThrusterInterfaceAUVDriver(
        short i2c_bus,
        int pico_i2c_address,
        const std::vector<ThrusterParameters>& thruster_parameters,
        const std::vector<std::vector<double>>& poly_coeffs);
    /**
     * @brief calls both 1) interpolate_forces_to_pwm() to
     * convert the thruster forces to PWM values and 2) send_data_to_escs() to
     * send them to the ESCs via I2C
     *
     * @param thruster_forces_array vector of forces for each thruster
     *
     * @return std::vector<uint16_t> vector of pwm values sent to each thruster
     */
    std::vector<uint16_t> drive_thrusters(
        const std::vector<double>& thruster_forces_array);

   private:
    int bus_fd_;  ///< file descriptor for the I2C bus (integer >0 that uniquely
                  ///< identifies the device. -1 if it fails)

    int i2c_bus_;
    int pico_i2c_address_;
    std::vector<ThrusterParameters> thruster_parameters_;
    std::vector<std::vector<double>> poly_coeffs_;

    uint16_t idle_pwm_value_;  ///< pwm value when force = 0.00

    /**
     * @brief only take the thruster forces and return PWM values
     *
     * @param thruster_forces_array vector of forces for each thruster
     *
     * @return std::vector<uint16_t> vector of pwm values sent to each thruster
     * if we want to publish them for debug purposes
     */
    std::vector<uint16_t> interpolate_forces_to_pwm(
        const std::vector<double>& thruster_forces_array);

    /**
     * @brief scalar map from force to pwm x->y. Choose coefficients [LEFT] or
     * [RIGHT] based on sign(force)
     *
     * @param force  scalar force value
     * @param coeffs std::vector<std::vector<double>> coeffs contains the pair
     * of coefficients
     *
     * @return std::uint16_t scalar pwm value
     */
    std::uint16_t force_to_pwm(double force,
                               const std::vector<std::vector<double>>& coeffs);

    /**
     * @brief compute y = a*x^3 + b*x^2 + c*x + d
     * @param force x
     * @param coeffs a,b,c,d
     *
     * @return std::uint16_t pwm value
     */
    std::uint16_t calc_poly(double force, const std::vector<double>& coeffs);

    /**
     * @brief only takes the pwm values computed and sends them
     * to the ESCs via I2C
     *
     * @param thruster_pwm_array vector of pwm values to send
     */
    void send_data_to_escs(const std::vector<uint16_t>& thruster_pwm_array);

    /**
     * @brief convert Newtons to Kg
     *
     * @param force Newtons
     *
     * @return double Kg
     */
    static constexpr double to_kg(double force) { return force / 9.80665; }

    /**
     * @brief convert pwm values to i2c bytes
     *
     * @param pwm pwm value
     *
     * @return std::array<std::uint8_t, 2> i2c data
     */
    static constexpr std::array<std::uint8_t, 2> pwm_to_i2c_data(
        std::uint16_t pwm) {
        return {static_cast<std::uint8_t>((pwm >> 8) & 0xFF),
                static_cast<std::uint8_t>(pwm & 0xFF)};
    }
};

#endif  // THRUSTER_INTERFACE_AUV_DRIVER_HPP
