#include "vortex/simulator/stonefish/simulator.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/sensors/Sample.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/Odometry.h>
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

    const bool parsed = parser.Parse(scenario_path_);

    if (!parsed) {
        std::cerr
            << "[Stonefish] Failed to parse scenario: "
            << scenario_path_ << '\n';

        return;
    }

    std::cout << "[Stonefish] Scenario parsed successfully\n";

    cache_thrusters();
    dump_sensors();
}

void VortexSimulationManager::dump_sensors() {
    unsigned int sensor_id = 0;
    sf::Sensor* sensor = nullptr;

    std::cerr << "[Stonefish] Sensors:\n";

    while ((sensor = getSensor(sensor_id++)) != nullptr) {
        std::cerr << "  sensor[" << sensor_id - 1 << "] "
                  << "name=" << sensor->getName()
                  << " type=" << static_cast<int>(sensor->getType()) << '\n';
    }
}

void VortexSimulationManager::cache_thrusters() {
    thrusters_.fill(nullptr);

    auto* robot = getRobot("nautilus");

    if (!robot) {
        std::cerr << "[Stonefish] Missing robot: nautilus\n";
        return;
    }

    unsigned int actuator_id = 0;
    unsigned int thruster_id = 0;
    sf::Actuator* actuator = nullptr;

    while ((actuator = robot->getActuator(actuator_id++)) != nullptr) {
        std::cerr << "[Stonefish] actuator[" << actuator_id - 1
                  << "] name=" << actuator->getName() << '\n';

        if (actuator->getType() != sf::ActuatorType::THRUSTER) {
            continue;
        }

        auto* thruster = dynamic_cast<sf::Thruster*>(actuator);

        if (!thruster) {
            std::cerr
                << "[Stonefish] actuator reports THRUSTER but cast failed: "
                << actuator->getName() << '\n';
            continue;
        }

        if (thruster_id >= thrusters_.size()) {
            std::cerr << "[Stonefish] Extra thruster ignored: "
                      << actuator->getName() << '\n';
            continue;
        }

        thrusters_[thruster_id] = thruster;

        std::cerr << "[Stonefish] cached thruster " << thruster_id + 1 << ": "
                  << actuator->getName() << '\n';

        ++thruster_id;
    }

    std::cerr << "[Stonefish] cached " << thruster_id << " thrusters\n";
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
    for (std::size_t i = 0; i < thrusters_.size(); ++i) {
        auto* thruster = thrusters_[i];

        if (!thruster) {
            continue;
        }

        const double normalized = std::clamp(command.command[i], -1.0, 1.0);
        const double setpoint =
            normalized * static_cast<double>(thruster->getSetpointLimit());

        thruster->setSetpoint(static_cast<sf::Scalar>(setpoint));
    }
}

ImuReading VortexSimulationManager::read_imu(const std::string& name) {
    ImuReading out{};

    auto* sensor = getSensor(name);
    auto* imu = dynamic_cast<sf::IMU*>(sensor);

    if (imu == nullptr) {
        return out;
    }

    const sf::Sample sample = imu->getLastSample();

    if (sample.getNumOfDimensions() != 9U) {
        return out;
    }

    out.timestamp_s = static_cast<double>(sample.getTimestamp());

    out.angular_velocity_rad_s = {
        static_cast<double>(sample.getValue(3)),
        static_cast<double>(sample.getValue(4)),
        static_cast<double>(sample.getValue(5)),
    };

    out.linear_acceleration_m_s2 = {
        static_cast<double>(sample.getValue(6)),
        static_cast<double>(sample.getValue(7)),
        static_cast<double>(sample.getValue(8)),
    };

    /*
     * Stonefish returns roll, pitch and yaw, not a quaternion.
     * Convert these properly rather than returning identity.
     */
    const double roll = static_cast<double>(sample.getValue(0));
    const double pitch = static_cast<double>(sample.getValue(1));
    const double yaw = static_cast<double>(sample.getValue(2));

    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    out.orientation_xyzw = {
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    };

    out.valid = true;
    return out;
}

PressureReading VortexSimulationManager::read_pressure(
    const std::string& name) {
    PressureReading out{};
    out.timestamp_s = time_s();

    auto* sensor = getSensor(name);
    auto* pressure = dynamic_cast<sf::Pressure*>(sensor);

    if (pressure == nullptr) {
        return out;
    }

    constexpr unsigned int pressure_channel = 0U;

    out.pressure_pa =
        static_cast<double>(pressure->getLastValue(pressure_channel));

    out.valid = true;
    return out;
}

DvlReading VortexSimulationManager::read_dvl(
    const std::string& name) {
    DvlReading out{};

    auto* sensor = getSensor(name);
    auto* dvl = dynamic_cast<sf::DVL*>(sensor);

    if (dvl == nullptr) {
        return out;
    }

    const sf::Sample sample = dvl->getLastSample();

    constexpr std::size_t expected_dimensions = 8U;

    if (sample.getNumOfDimensions() != expected_dimensions) {
        return out;
    }

    out.timestamp_s =
        static_cast<double>(sample.getTimestamp());

    out.velocity_body_m_s = {
        static_cast<double>(sample.getValue(0)),
        static_cast<double>(sample.getValue(1)),
        static_cast<double>(sample.getValue(2)),
    };

    out.altitude_m =
        static_cast<double>(sample.getValue(3));

    const auto status =
        static_cast<unsigned int>(sample.getValue(7));

    const bool bottom_ping_valid =
        status == 0U || status == 2U;

    out.beam_valid = {
        bottom_ping_valid,
        bottom_ping_valid,
        bottom_ping_valid,
        bottom_ping_valid,
    };

    out.velocity_valid = bottom_ping_valid;
    out.altitude_valid = bottom_ping_valid;

    /*
     * The sensor exists and returned a correctly shaped sample.
     * Per-measurement validity is represented separately above.
     */
    out.valid = true;

    return out;
}

CameraFrame VortexSimulationManager::read_camera(const std::string& name) {
    CameraFrame out{};
    out.timestamp_s = time_s();

    auto* sensor = getSensor(name);
    auto* camera = dynamic_cast<sf::Camera*>(sensor);

    if (camera == nullptr) {
        return out;
    }

    camera->getResolution(out.width, out.height);

    if (out.width == 0U || out.height == 0U) {
        return out;
    }

    constexpr std::uint32_t rgb_channels = 3U;
    out.channels = rgb_channels;

    const void* raw_image_data = camera->getImageDataPointer();

    if (raw_image_data == nullptr) {
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

SonarFrame VortexSimulationManager::read_sonar(
    const std::string& name) {
    SonarFrame out{};
    out.timestamp_s = time_s();

    auto* sensor = getSensor(name);
    auto* fls = dynamic_cast<sf::FLS*>(sensor);

    if (fls == nullptr) {
        return out;
    }

    unsigned int beams = 0U;
    unsigned int bins = 0U;

    fls->getResolution(beams, bins);

    if (beams == 0U || bins == 0U) {
        return out;
    }

    const void* raw_data = fls->getImageDataPointer();

    if (raw_data == nullptr) {
        return out;
    }

    const auto* samples =
        static_cast<const std::uint8_t*>(raw_data);

    const std::size_t sample_count =
        static_cast<std::size_t>(beams) *
        static_cast<std::size_t>(bins);

    out.beams = static_cast<std::uint32_t>(beams);
    out.bins = static_cast<std::uint32_t>(bins);

    out.intensities.resize(sample_count);

    for (std::size_t i = 0; i < sample_count; ++i) {
        out.intensities[i] =
            static_cast<float>(samples[i]) / 255.0F;
    }

    out.valid = true;
    return out;
}

OdometryReading VortexSimulationManager::read_odometry(
    const std::string& name) {
    OdometryReading out{};

    auto* sensor = getSensor(name);
    auto* odometry = dynamic_cast<sf::Odometry*>(sensor);

    if (odometry == nullptr) {
        return out;
    }

    const sf::Sample sample = odometry->getLastSample();

    constexpr std::size_t expected_dimensions = 13U;

    if (sample.getNumOfDimensions() != expected_dimensions) {
        return out;
    }

    out.timestamp_s =
        static_cast<double>(sample.getTimestamp());

    out.position_world_m = {
        static_cast<double>(sample.getValue(0)),
        static_cast<double>(sample.getValue(1)),
        static_cast<double>(sample.getValue(2)),
    };

    out.velocity_body_m_s = {
        static_cast<double>(sample.getValue(3)),
        static_cast<double>(sample.getValue(4)),
        static_cast<double>(sample.getValue(5)),
    };

    out.orientation_xyzw = {
        static_cast<double>(sample.getValue(6)),
        static_cast<double>(sample.getValue(7)),
        static_cast<double>(sample.getValue(8)),
        static_cast<double>(sample.getValue(9)),
    };

    out.angular_velocity_body_rad_s = {
        static_cast<double>(sample.getValue(10)),
        static_cast<double>(sample.getValue(11)),
        static_cast<double>(sample.getValue(12)),
    };

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
