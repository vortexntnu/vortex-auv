#ifndef THRUSTER_INTERFACE_AUV__THRUSTER_INTERFACE_AUV_DRIVER_HPP_
#define THRUSTER_INTERFACE_AUV__THRUSTER_INTERFACE_AUV_DRIVER_HPP_

#include <vortex/io/can/can_interface.hpp>

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <linux/can.h>
namespace vortex::propulsion {

struct ThrusterParameters {
    std::uint8_t mapping;
    std::int8_t direction;
    std::uint16_t pwm_min;
    std::uint16_t pwm_max;
};

enum PolySide { LEFT = 0, RIGHT = 1 };

using FaultEventCallback =
    std::function<void(std::uint8_t channel, std::uint8_t code)>;

using PGoodEventCallback =
    std::function<void(std::uint8_t channel, std::uint8_t code)>;

using KillswitchEventCallback = std::function<void()>;

using CurrentMeasurementsCallback =
    std::function<void(const std::array<float, 8>& currents)>;

class ThrusterInterface {
   public:
    ~ThrusterInterface();

    ThrusterInterface(
        const std::string& can_interface_name,
        const std::vector<ThrusterParameters>& thruster_parameters,
        const std::vector<double>& right_coeffs,
        const std::vector<double>& left_coeffs);

    int init_can();

    [[nodiscard]]
    std::optional<std::vector<std::uint16_t>> drive_thrusters(
        const Eigen::Ref<const Eigen::VectorXd>& thruster_forces);

    int set_camera_light(float percentage);

    void set_fault_event_callback(FaultEventCallback callback);
    void set_pgood_event_callback(PGoodEventCallback callback);
    void set_killswitch_event_callback(KillswitchEventCallback callback);
    void set_current_measurements_callback(
        CurrentMeasurementsCallback callback);

   private:
    std::vector<std::uint16_t> interpolate_forces_to_pwm(
        const Eigen::Ref<const Eigen::VectorXd>& mapped_forces);

    std::uint16_t force_to_pwm(double force);

    std::uint16_t calc_poly(double force, const std::vector<double>& coeffs);

    int send_data_to_escs(const std::vector<std::uint16_t>& thruster_pwm_array);
    void handle_can_frame(const struct canfd_frame& frame,
                          vortex::io::can::CanStatus status);

    static constexpr double to_kg(double force) { return force / 9.80665; }

   private:
    std::string can_interface_name_;
    vortex::io::can::CanInterface can_;

    std::vector<ThrusterParameters> thruster_parameters_;
    std::vector<double> right_coeffs_;
    std::vector<double> left_coeffs_;
    std::uint16_t idle_pwm_value_{1500};

    FaultEventCallback fault_event_callback_;
    PGoodEventCallback pgood_event_callback_;
    KillswitchEventCallback killswitch_event_callback_;
    CurrentMeasurementsCallback current_measurements_callback_;
};

}  // namespace vortex::propulsion

#endif  // THRUSTER_INTERFACE_AUV__THRUSTER_INTERFACE_AUV_DRIVER_HPP_
