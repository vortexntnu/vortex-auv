#pragma once

#include <chrono>

#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller.hpp"
#include "eskf/eskf.hpp"
#include "reference_filter_dp_quat/waypoint_guidance_manager.hpp"
#include "vortex/propulsion/thrust_allocator/thrust_allocator.hpp"


struct VehicleConfig {
    vortex::guidance::WaypointGuidanceManagerConfig guidance;
    vortex::control::DPAdaptParams controller;
    vortex::propulsion::ThrustAllocatorSettings allocator;

    EskfParams eskf;

    Eigen::Matrix3d dvl_measurement_noise;
    double pressure_measurement_noise_pa2;
};
