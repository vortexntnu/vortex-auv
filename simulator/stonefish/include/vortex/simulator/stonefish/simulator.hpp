#pragma once

#include <memory>
#include <string>

#include "vortex/simulator/stonefish/actuator_commands.hpp"
#include "vortex/simulator/stonefish/sensor_data.hpp"


#include <Stonefish/core/SimulationManager.h>

namespace vortex::simulation::stonefish {

class StonefishSimulator {
public:
  StonefishSimulator(std::string scenario_path, double simulation_frequency_hz);

  void initialise();
  void step();

  double time_s() const;

  void set_thrusters(const ThrusterCommand &command);

  ImuReading read_imu(const std::string &name) const;
  PressureReading read_pressure(const std::string &name) const;
  DvlReading read_dvl(const std::string &name) const;
  CameraFrame read_camera(const std::string &name) const;
  SonarFrame read_sonar(const std::string &name) const;

private:
  std::string scenario_path_;
  double simulation_frequency_hz_{100.0};
  double dt_s_{0.01};

  std::unique_ptr<sf::SimulationManager> sim_;
};

} // namespace vortex::simulation::stonefish
