add_executable(vehicle_control
  main.cpp
)

target_link_libraries(vehicle_control
  PRIVATE
    vortex::can_interface
    vortex::nucleus_interface
    vortex::stonefish
    thrust_allocator
    thruster_interface
    dp_adapt_backs_controller_quat
    reference_filter_dp_quat
)

if(VORTEX_ENABLE_TRACY)
  target_link_libraries(vehicle_control
    PRIVATE
      Tracy::TracyClient
  )
endif()
