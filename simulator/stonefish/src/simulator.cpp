#include "vortex/simulator/stonefish/simulator.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <Stonefish/core/ScenarioParser.h>

#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/vision/Camera.h>
#include <Stonefish/sensors/vision/FLS.h>

namespace vortex::simulation::stonefish {

VortexSimulationManager::VortexSimulationManager(sf::Scalar steps_per_second,
                                                 std::string scenario_path,
                                                 StepCallback step_callback)
    : sf::SimulationManager(steps_per_second),
      scenario_path_(std::move(scenario_path)),
      step_callback_(std::move(step_callback)) {}

void VortexSimulationManager::BuildScenario() {
    sf::ScenarioParser parser(this);
    parser.Parse(scenario_path_);
}

void VortexSimulationManager::SimulationStepCompleted(sf::Scalar dt) {
    if (step_callback_) {
        step_callback_(*this, static_cast<double>(dt));
    }
}

double VortexSimulationManager::time_s() const {
    return static_cast<double>(getSimulationTime());
}

void VortexSimulationManager::set_thrusters(const ThrusterCommand& command) {
    for (std::size_t i = 0; i < ThrusterCommand::kNumThrusters; ++i) {
        const std::string name = "Thruster" + std::to_string(i + 1);

        auto* actuator = getActuator(name);
        auto* thruster = dynamic_cast<sf::Thruster*>(actuator);

        if (!thruster) {
            std::cerr << "Warning: could not find thruster actuator: " << name
                      << '\n';
            continue;
        }

        const double u = std::clamp(command.command[i], -1.0, 1.0);
        thruster->setSetpoint(static_cast<sf::Scalar>(u));
    }
}

ImuReading VortexSimulationManager::read_imu(const std::string& name) {
    ImuReading out{};
    out.timestamp_s = time_s();

    auto* sensor = getSensor(name);
    auto* imu = dynamic_cast<sf::IMU*>(sensor);

    if (!imu) {
        return out;
    }

    sf::Vector3 linear_velocity;
    sf::Vector3 angular_velocity;
    imu->getSensorVelocity(linear_velocity, angular_velocity);

    out.angular_velocity_rad_s =
        std::array<double, 3>{static_cast<double>(angular_velocity.x()),
                              static_cast<double>(angular_velocity.y()),
                              static_cast<double>(angular_velocity.z())};

    out.linear_acceleration_m_s2 = std::array<double, 3>{0.0, 0.0, 0.0};
    out.orientation_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};

    out.valid = true;
    return out;
}

PressureReading VortexSimulationManager::read_pressure(
    const std::string& name) {
    PressureReading out{};
    out.timestamp_s = time_s();

    auto* sensor = getSensor(name);
    auto* pressure = dynamic_cast<sf::Pressure*>(sensor);

    if (!pressure) {
        return out;
    }

    // TODO: Implement using actual Pressure.h API.
    out.valid = false;
    return out;
}

DvlReading VortexSimulationManager::read_dvl(const std::string& name) {
    DvlReading out{};
    out.timestamp_s = time_s();

    auto* sensor = getSensor(name);
    auto* dvl = dynamic_cast<sf::DVL*>(sensor);

    if (!dvl) {
        return out;
    }

    sf::Vector3 linear_velocity;
    sf::Vector3 angular_velocity;
    dvl->getSensorVelocity(linear_velocity, angular_velocity);

    out.velocity_body_m_s =
        std::array<double, 3>{static_cast<double>(linear_velocity.x()),
                              static_cast<double>(linear_velocity.y()),
                              static_cast<double>(linear_velocity.z())};

    out.beam_valid = std::array<bool, 4>{false, false, false, false};

    out.valid = true;
    return out;
}

CameraFrame VortexSimulationManager::read_camera(const std::string& name) {
    CameraFrame out{};
    out.timestamp_s = time_s();

    auto* sensor = getSensor(name);
    auto* camera = dynamic_cast<sf::Camera*>(sensor);

    if (!camera) {
        return out;
    }

    camera->getResolution(out.width, out.height);
    out.channels = 3;

    const void* raw_image_data = camera->getImageDataPointer();

    if (!raw_image_data) {
        return out;
    }

    const auto* image_data = static_cast<const std::uint8_t*>(raw_image_data);

    const std::size_t byte_count = static_cast<std::size_t>(out.width) *
                                   static_cast<std::size_t>(out.height) *
                                   static_cast<std::size_t>(out.channels);

    out.pixels.assign(image_data, image_data + byte_count);

    out.valid = true;
    return out;
}

SonarFrame VortexSimulationManager::read_sonar(const std::string& name) {
    SonarFrame out{};
    out.timestamp_s = time_s();

    auto* sensor = getSensor(name);
    auto* fls = dynamic_cast<sf::FLS*>(sensor);

    if (!fls) {
        return out;
    }

    unsigned int width = 0;
    unsigned int height = 0;
    fls->getDisplayResolution(width, height);

    if (width == 0 || height == 0) {
        return out;
    }

    void* raw_data = fls->getImageDataPointer();

    if (!raw_data) {
        return out;
    }

    const auto* data = static_cast<const GLubyte*>(raw_data);

    const std::size_t count =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

    out.beams = static_cast<std::uint32_t>(width);
    out.bins = static_cast<std::uint32_t>(height);

    out.intensities.resize(count);

    for (std::size_t i = 0; i < count; ++i) {
        out.intensities[i] = static_cast<float>(data[i]) / 255.0f;
    }

    out.valid = true;
    return out;
}

StonefishSimulator::StonefishSimulator(std::string scenario_path,
                                       std::string data_path,
                                       double simulation_frequency_hz)
    : scenario_path_(std::move(scenario_path)),
      data_path_(std::move(data_path)),
      simulation_frequency_hz_(simulation_frequency_hz) {}

void StonefishSimulator::set_step_callback(
    VortexSimulationManager::StepCallback callback) {
    step_callback_ = std::move(callback);
}

void StonefishSimulator::run_graphical() {
    sf::RenderSettings render_settings;
    sf::HelperSettings helper_settings;

    manager_ = std::make_unique<VortexSimulationManager>(
        static_cast<sf::Scalar>(simulation_frequency_hz_), scenario_path_,
        step_callback_);

    app_ = std::make_unique<sf::GraphicalSimulationApp>(
        "Vortex Stonefish standalone", data_path_, render_settings,
        helper_settings, manager_.get());

    app_->Run();
}

VortexSimulationManager& StonefishSimulator::manager() {
    if (!manager_) {
        throw std::runtime_error("Stonefish simulator has not been started");
    }

    return *manager_;
}

}  // namespace vortex::simulation::stonefish
