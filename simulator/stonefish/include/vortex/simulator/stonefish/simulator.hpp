#pragma once

#include <functional>
#include <memory>
#include <string>

#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/actuators/Thruster.h>

#include "vortex/simulator/stonefish/actuator_commands.hpp"
#include "vortex/simulator/stonefish/sensor_data.hpp"

namespace vortex::simulation::stonefish {

class VortexSimulationManager final : public sf::SimulationManager {
   public:
    using StepCallback = std::function<void(VortexSimulationManager&, double)>;

    VortexSimulationManager(sf::Scalar steps_per_second,
                            std::string scenario_path,
                            StepCallback step_callback = nullptr);

    void BuildScenario() override;

    void cache_thrusters();
    void SimulationStepCompleted(sf::Scalar dt) override;

    double time_s() const;

    void set_thrusters(const ThrusterCommand& command);

    ImuReading read_imu(const std::string& name);
    PressureReading read_pressure(const std::string& name);
    DvlReading read_dvl(const std::string& name);
    CameraFrame read_camera(const std::string& name);
    SonarFrame read_sonar(const std::string& name);

   private:

    std::array<sf::Thruster*, 8> thrusters_{};
    std::string scenario_path_;
    StepCallback step_callback_;
};

class StonefishSimulator {
   public:
    StonefishSimulator(std::string scenario_path,
                       std::string data_path,
                       double simulation_frequency_hz);

    void set_step_callback(VortexSimulationManager::StepCallback callback);

    void run_graphical();

    VortexSimulationManager& manager();

   private:
    std::string scenario_path_;
    std::string data_path_;
    double simulation_frequency_hz_{500.0};

    VortexSimulationManager::StepCallback step_callback_;

    std::unique_ptr<VortexSimulationManager> manager_;
    std::unique_ptr<sf::GraphicalSimulationApp> app_;
};

}  // namespace vortex::simulation::stonefish
