#include "vortex/simulator/stonefish/simulator.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationManager.h>

#include <Stonefish/core/Robot.h>

#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/vision/Camera.h>
#include <Stonefish/sensors/vision/FLS.h>

#include <Stonefish/actuators/Thruster.h>

namespace vortex::simulation::stonefish {

class VortexSimulationManager final : public sf::SimulationManager {
public:
  VortexSimulationManager(sf::Scalar steps_per_second,
                          std::string scenario_path)
      : sf::SimulationManager(steps_per_second),
        scenario_path_(std::move(scenario_path)) {}

  void BuildScenario() override {
    sf::ScenarioParser parser(this);
    parser.Parse(scenario_path_);
  }

private:
  std::string scenario_path_;
};

} // namespace vortex::simulation::stonefish

namespace vortex::simulation::stonefish {

StonefishSimulator::StonefishSimulator(std::string scenario_path,
                                       double simulation_frequency_hz)
    : scenario_path_(std::move(scenario_path)),
      simulation_frequency_hz_(simulation_frequency_hz),
      dt_s_(1.0 / simulation_frequency_hz) {}

void StonefishSimulator::initialise() {
  if (scenario_path_.empty()) {
    throw std::runtime_error("scenario_path is empty");
  }

  if (simulation_frequency_hz_ <= 0.0) {
    throw std::runtime_error("simulation_frequency_hz must be positive");
  }

  sim_ = std::make_unique<VortexSimulationManager>(
      static_cast<sf::Scalar>(simulation_frequency_hz_), scenario_path_);

  sim_->BuildScenario();
}

void StonefishSimulator::step() {
  if (!sim_) {
    throw std::runtime_error("StonefishSimulator::initialise() was not called");
  }

  sim_->AdvanceSimulation();
}

double StonefishSimulator::time_s() const {
  if (!sim_) {
    return 0.0;
  }

  return static_cast<double>(sim_->getSimulationTime());
}

void StonefishSimulator::set_thrusters(const ThrusterCommand &command) {
  if (!sim_) {
    throw std::runtime_error("StonefishSimulator::initialise() was not called");
  }

  for (std::size_t i = 0; i < ThrusterCommand::kNumThrusters; ++i) {
    const std::string name = "Thruster" + std::to_string(i + 1);

    auto *actuator = sim_->getActuator(name);
    auto *thruster = dynamic_cast<sf::Thruster *>(actuator);

    if (!thruster) {
      std::cerr << "Warning: could not find thruster actuator: " << name
                << '\n';
      continue;
    }

    const double u = std::clamp(command.command[i], -1.0, 1.0);

    thruster->setSetpoint(static_cast<sf::Scalar>(u));
  }
}

ImuReading StonefishSimulator::read_imu(const std::string &name) const {
  ImuReading out{};
  out.timestamp_s = time_s();

  if (!sim_) {
    return out;
  }

  auto *sensor = sim_->getSensor(name);
  auto *imu = dynamic_cast<sf::IMU *>(sensor);

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

  // Temporary until we use the real IMU measurement API.
  out.linear_acceleration_m_s2 = std::array<double, 3>{0.0, 0.0, 0.0};

  // x, y, z, w identity quaternion.
  out.orientation_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};

  out.valid = true;
  return out;
}

PressureReading
StonefishSimulator::read_pressure(const std::string &name) const {
  PressureReading out{};
  out.timestamp_s = time_s();

  if (!sim_) {
    return out;
  }

  auto *sensor = sim_->getSensor(name);
  auto *pressure = dynamic_cast<sf::Pressure *>(sensor);

  if (!pressure) {
    return out;
  }

  // TODO: Replace with actual sf::Pressure API after inspecting Pressure.h.
  out.valid = false;
  return out;
}

DvlReading StonefishSimulator::read_dvl(const std::string &name) const {
  DvlReading out{};
  out.timestamp_s = time_s();

  if (!sim_) {
    return out;
  }

  auto *sensor = sim_->getSensor(name);
  auto *dvl = dynamic_cast<sf::DVL *>(sensor);

  if (!dvl) {
    return out;
  }

  sf::Vector3 linear_velocity;
  sf::Vector3 angular_velocity;
  dvl->getSensorVelocity(linear_velocity, angular_velocity);

  // This is only the sensor-frame velocity, not necessarily the DVL
  // measurement.
  out.velocity_body_m_s =
      std::array<double, 3>{static_cast<double>(linear_velocity.x()),
                            static_cast<double>(linear_velocity.y()),
                            static_cast<double>(linear_velocity.z())};

  out.beam_valid = std::array<bool, 4>{false, false, false, false};

  out.valid = true;
  return out;
}

CameraFrame StonefishSimulator::read_camera(const std::string &name) const {
  CameraFrame out{};
  out.timestamp_s = time_s();

  if (!sim_) {
    return out;
  }

  auto *sensor = sim_->getSensor(name);
  auto *camera = dynamic_cast<sf::Camera *>(sensor);

  if (!camera) {
    return out;
  }

  /*
   * Vision sensors may require the graphical/rendering pipeline.
   * Exact API depends on whether you run console or graphical mode.
   */

  camera->getResolution(out.width, out.height);
  out.channels = 3;

  const void *raw_image_data = camera->getImageDataPointer();

  if (!raw_image_data) {
    return out;
  }

  const auto *image_data = static_cast<const std::uint8_t *>(raw_image_data);

  const std::size_t byte_count = static_cast<std::size_t>(out.width) *
                                 static_cast<std::size_t>(out.height) *
                                 static_cast<std::size_t>(out.channels);

  out.pixels.assign(image_data, image_data + byte_count);

  out.valid = true;
  return out;
}

SonarFrame StonefishSimulator::read_sonar(const std::string &name) const {
  SonarFrame out{};
  out.timestamp_s = time_s();

  if (!sim_) {
    return out;
  }

  auto *sensor = sim_->getSensor(name);
  auto *fls = dynamic_cast<sf::FLS *>(sensor);

  if (!fls) {
    return out;
  }

  unsigned int width = 0;
  unsigned int height = 0;
  fls->getDisplayResolution(width, height);

  if (width == 0 || height == 0) {
    return out;
  }

  void *raw_data = fls->getImageDataPointer();

  if (!raw_data) {
    return out;
  }

  const auto *data = static_cast<const GLubyte *>(raw_data);

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

} // namespace vortex::simulation::stonefish
