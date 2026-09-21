find_package(launch_testing_ament_cmake REQUIRED)

add_executable(test_velocity_controller_integration
    test_velocity_controller_integration.cpp
)
target_include_directories(test_velocity_controller_integration PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(test_velocity_controller_integration
    rclcpp
    geometry_msgs
    nav_msgs
    vortex_msgs
)
target_link_libraries(test_velocity_controller_integration
    gtest
    gtest_main
)

install(TARGETS test_velocity_controller_integration
    DESTINATION lib/${PROJECT_NAME}
)

add_launch_test(
    test_velocity_controller_launch.py
    TARGET velocity_controller_integration_test
    TIMEOUT 60
)