add_library(vehicle_control_core
  control/vehicle_control.cpp
  config/nautilus_config.cpp
)

target_link_libraries(vehicle_control_core
  PUBLIC
    vortex::eskf
    thrust_allocator
    dp_adapt_backs_controller_quat
    reference_filter_dp_quat
)

target_include_directories(vehicle_control_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(vehicle_control_simulator
  simulator/sim_main.cpp
  simulator/stonefish_io.cpp
)

target_link_libraries(vehicle_control_simulator
  PRIVATE
    vehicle_control_core
    vortex::stonefish
)

add_executable(vehicle_control_hardware
  hardware/hardware_main.cpp
  hardware/hardware_io.cpp
)

target_link_libraries(vehicle_control_hardware
  PRIVATE
    vehicle_control_core
    vortex::can_interface
    vortex::nucleus_interface
    thruster_interface
)

if(VORTEX_ENABLE_TRACY)
  target_link_libraries(vehicle_control_core
    PUBLIC
      Tracy::TracyClient
  )
endif()
