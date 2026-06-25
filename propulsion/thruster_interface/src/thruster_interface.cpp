#include "vortex/propulsion/thruster_interface/thruster_interface.hpp"
#include <vortex/io/can/can_interface.hpp>

#include <unistd.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <thread>

namespace vortex::propulsion {

static constexpr std::uint32_t CAN_ID_DISABLE_THRUSTERS = 0x369U;
static constexpr std::uint32_t CAN_ID_ENABLE_THRUSTERS = 0x36AU;
static constexpr std::uint32_t CAN_ID_RESET = 0x3BAU;
static constexpr std::uint32_t CAN_ID_SET_THRUSTERS_PWM = 0x36CU;
static constexpr std::uint32_t CAN_ID_SET_LIGHT_PWM = 0x36DU;

static constexpr std::uint32_t CAN_ID_FLT_EVENT = 0x36EU;
static constexpr std::uint32_t CAN_ID_PGOOD_EVENT = 0x36FU;
static constexpr std::uint32_t CAN_ID_KILLSWITCH_EVENT = 0x370U;
static constexpr std::uint32_t CAN_ID_CURRENT_MEASUREMENTS = 0x371U;

using vortex::io::can;

ThrusterInterface::ThrusterInterface(
    const std::string& can_interface_name,
    const std::vector<ThrusterParameters>& thruster_parameters,
    const std::vector<double>& right_coeffs,
    const std::vector<double>& left_coeffs)
    : can_interface_name_(can_interface_name),
      thruster_parameters_(thruster_parameters),
      right_coeffs_(right_coeffs),
      left_coeffs_(left_coeffs) {
    idle_pwm_value_ = static_cast<std::uint16_t>(
        (calc_poly(0.0, left_coeffs_) + calc_poly(0.0, right_coeffs_)) / 2);
}

ThrusterInterface::~ThrusterInterface() {
    if (can_.is_initialized()) {
        send_data_to_escs(std::vector<std::uint16_t>(
            thruster_parameters_.size(), idle_pwm_value_));
        can_.stop_async_receive();
    }
}

int ThrusterInterface::init_can() {
    can_status status = can_.init(can_interface_name_);
    if (status != can_status::OK) {
        return static_cast<int>(status);
    }

    struct can_filter filters[] = {
        {
            .can_id = CAN_ID_FLT_EVENT,
            .can_mask = CAN_SFF_MASK,
        },
        {
            .can_id = CAN_ID_PGOOD_EVENT,
            .can_mask = CAN_SFF_MASK,
        },
        {
            .can_id = CAN_ID_KILLSWITCH_EVENT,
            .can_mask = CAN_SFF_MASK,
        },
        {
            .can_id = CAN_ID_CURRENT_MEASUREMENTS,
            .can_mask = CAN_SFF_MASK,
        },
    };

    status = can_.set_filters(filters, sizeof(filters) / sizeof(filters[0]));
    if (status != can_status::OK) {
        return static_cast<int>(status);
    }

    status = can_.start_async_receive(
        [this](const struct canfd_frame& frame, can_status rx_status) {
            handle_can_frame(frame, rx_status);
        });

    if (status != can_status::OK) {
        return static_cast<int>(status);
    }

    return 0;
}

std::vector<std::uint16_t>
ThrusterInterface::interpolate_forces_to_pwm(
    const std::vector<double>& thruster_forces_array) {
    std::vector<std::uint16_t> pwm(thruster_forces_array.size());

    for (std::size_t i = 0; i < thruster_forces_array.size(); ++i) {
        const double force_in_kg = to_kg(thruster_forces_array[i]);
        pwm[i] = force_to_pwm(force_in_kg);
    }

    return pwm;
}

std::uint16_t ThrusterInterface::force_to_pwm(double force) {
    constexpr double deadband_kg = 0.03;

    if (std::abs(force) < deadband_kg) {
        return idle_pwm_value_;
    }

    if (force < 0.0) {
        return calc_poly(force, left_coeffs_);
    }

    return calc_poly(force, right_coeffs_);
}

std::uint16_t ThrusterInterface::calc_poly(
    double force,
    const std::vector<double>& coeffs) {
    if (coeffs.size() < 4) {
        return idle_pwm_value_;
    }

    return static_cast<std::uint16_t>(coeffs[0] * std::pow(force, 3) +
                                      coeffs[1] * std::pow(force, 2) +
                                      coeffs[2] * force + coeffs[3]);
}

std::optional<std::vector<std::uint16_t>>
ThrusterInterface::drive_thrusters(
    const std::vector<double>& thruster_forces_array) {
    if (thruster_forces_array.size() < thruster_parameters_.size()) {
        return std::nullopt;
    }

    std::vector<double> mapped_forces(thruster_parameters_.size());

    for (std::size_t i = 0; i < thruster_parameters_.size(); ++i) {
        const auto& param = thruster_parameters_[i];
        const std::size_t idx = param.mapping;

        if (idx >= thruster_forces_array.size()) {
            return std::nullopt;
        }

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

int ThrusterInterface::send_data_to_escs(
    const std::vector<std::uint16_t>& thruster_pwm_array) {
    if (!can_.is_initialized()) {
        return static_cast<int>(can_status::ERR_NOT_INITIALIZED);
    }

    if (thruster_pwm_array.size() != 8) {
        return -1;
    }

    std::array<std::uint8_t, 16> payload{};

    for (std::size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        const std::uint16_t value = thruster_pwm_array[i];

        payload[2 * i] = static_cast<std::uint8_t>(value & 0xFF);

        payload[2 * i + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
    }

    const can_status status =
        can_.send(CAN_ID_SET_THRUSTERS_PWM, payload.data(),
                  static_cast<std::uint8_t>(payload.size()),
                  true  // use BRS
        );

    if (status != can_status::OK) {
        return static_cast<int>(status);
    }

    return 0;
}

int ThrusterInterface::set_camera_light(float percentage) {
    if (!can_.is_initialized()) {
        return static_cast<int>(can_status::ERR_NOT_INITIALIZED);
    }

    percentage = std::clamp(percentage, 0.0f, 1.0f);

    const std::uint16_t pwm =
        static_cast<std::uint16_t>(1100.0f + 800.0f * percentage);

    std::array<std::uint8_t, 2> payload{};
    payload[0] = static_cast<std::uint8_t>(pwm & 0xFF);
    payload[1] = static_cast<std::uint8_t>((pwm >> 8) & 0xFF);

    const can_status status =
        can_.send(CAN_ID_SET_LIGHT_PWM, payload.data(),
                  static_cast<std::uint8_t>(payload.size()), true);

    if (status != can_status::OK) {
        return static_cast<int>(status);
    }

    return 0;
}

void ThrusterInterface::handle_can_frame(
    const struct canfd_frame& frame,
    can_status status) {
    if (status != can_status::OK) {
        return;
    }

    switch (frame.can_id) {
        case CAN_ID_FLT_EVENT: {
            if (frame.len != 2) {
                break;
            }

            const std::uint8_t channel = frame.data[0];
            const std::uint8_t code = frame.data[1];

            if (fault_event_callback_) {
                fault_event_callback_(channel, code);
            }

            break;
        }

        case CAN_ID_PGOOD_EVENT: {
            if (frame.len != 2) {
                break;
            }

            const std::uint8_t channel = frame.data[0];
            const std::uint8_t code = frame.data[1];

            if (pgood_event_callback_) {
                pgood_event_callback_(channel, code);
            }

            break;
        }

        case CAN_ID_KILLSWITCH_EVENT: {
            if (frame.len != 0) {
                break;
            }

            if (killswitch_event_callback_) {
                killswitch_event_callback_();
            }

            break;
        }

        case CAN_ID_CURRENT_MEASUREMENTS: {
            constexpr std::size_t num_currents = 8;
            constexpr std::size_t expected_length =
                num_currents * sizeof(float);

            if (frame.len != expected_length) {
                break;
            }

            std::array<float, num_currents> currents{};
            std::memcpy(currents.data(), frame.data, expected_length);

            if (current_measurements_callback_) {
                current_measurements_callback_(currents);
            }

            break;
        }

        default:
            break;
    }
}

void ThrusterInterface::set_fault_event_callback(
    FaultEventCallback callback) {
    fault_event_callback_ = std::move(callback);
}

void ThrusterInterface::set_pgood_event_callback(
    PGoodEventCallback callback) {
    pgood_event_callback_ = std::move(callback);
}

void ThrusterInterface::set_killswitch_event_callback(
    KillswitchEventCallback callback) {
    killswitch_event_callback_ = std::move(callback);
}

void ThrusterInterface::set_current_measurements_callback(
    CurrentMeasurementsCallback callback) {
    current_measurements_callback_ = std::move(callback);
}

} // namespace vortex::propulsion
