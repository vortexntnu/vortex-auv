find_package(launch_testing_ament_cmake REQUIRED)

# Selve C++-testklienten som launch-filen starter som egen prosess.
# NB: IKKE ament_add_gtest her - den skal kjøres og styres av launch_testing
# sin Python-launch-fil (se test_velocity_controller_launch.py), ikke direkte
# av CTest. Bruk vanlig add_executable + install, slik at launch-filen kan
# finne og starte den som en frittstående prosess.
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

# Registrerer selve launch_testing-testen. Denne kjøres via
# `colcon test --packages-select velocity_controller` sammen med de
# vanlige enhetstestene, men er implementert som en separat CTest-entry
# siden launch_testing_add_test har sin egen kjøremekanikk under panseret.
add_launch_test(
    test_velocity_controller_launch.py
    TARGET velocity_controller_integration_test
    TIMEOUT 60
)