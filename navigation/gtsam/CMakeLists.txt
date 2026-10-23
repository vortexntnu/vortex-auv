cmake_minimum_required(VERSION 3.16)
project(gtsam_navigation VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Retain the external GTSAM prefix when using the package-local dependency build.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
find_package(ament_cmake REQUIRED)
find_package(GTSAM 4.3.0 EXACT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core SHARED src/estimator.cpp src/dvl_factor.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC gtsam Eigen3::Eigen)
set_target_properties(${PROJECT_NAME}_core PROPERTIES
  INSTALL_RPATH "$<TARGET_FILE_DIR:gtsam>")
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(${PROJECT_NAME}_component SHARED src/navigation_node.cpp)
target_link_libraries(${PROJECT_NAME}_component PUBLIC ${PROJECT_NAME}_core)
set_target_properties(${PROJECT_NAME}_component PROPERTIES
  INSTALL_RPATH "$ORIGIN;$<TARGET_FILE_DIR:gtsam>")
ament_target_dependencies(${PROJECT_NAME}_component PUBLIC
  rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs diagnostic_msgs tf2 tf2_ros)
target_compile_options(${PROJECT_NAME}_component PRIVATE -Wall -Wextra -Wpedantic)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "gtsam_navigation::NavigationNode"
  EXECUTABLE gtsam_navigation_node)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config scenarios DESTINATION share/${PROJECT_NAME})
install(FILES README.md SIMULATION_AUDIT.md STONEFISH_INTEGRATION.md
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/simulate_sensors.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/sensor_model.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/stonefish_sensors.py scripts/stonefish_control.py
  scripts/stonefish_evaluation.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/stonefish_math.py DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(GTSAM Eigen3 rclcpp rclcpp_components sensor_msgs
  geometry_msgs nav_msgs diagnostic_msgs tf2 tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(estimator_tests test/estimator_tests.cpp TIMEOUT 180)
  target_link_libraries(estimator_tests ${PROJECT_NAME}_core)
  # Explicit offline investigation tool, not a ROS node or installed interface.
  add_executable(audit_replay test/audit_replay.cpp)
  target_link_libraries(audit_replay ${PROJECT_NAME}_core)
  ament_add_pytest_test(sensor_model_tests test/test_sensor_model.py)
  ament_add_pytest_test(stonefish_math_tests test/test_stonefish_math.py)
  add_launch_test(test/test_navigation_launch.py TIMEOUT 60
    ENV "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=167" "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
endif()
ament_package()
