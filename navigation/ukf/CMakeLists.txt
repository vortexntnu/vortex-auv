cmake_minimum_required(VERSION 3.16)
project(ukf LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(spdlog REQUIRED)

add_library(ukf_core
  src/lie/SO3.cpp
  src/models/strapdown_ins.cpp
  src/models/measurement_models.cpp
  src/transforms/unscented_transform.cpp
  src/filters/ukf.cpp
)
set_target_properties(ukf_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ukf_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ukf_core PUBLIC Eigen3::Eigen)
ament_target_dependencies(ukf_core PUBLIC vortex_utils)

add_library(ukf_component SHARED src/ros/ukf_ros.cpp)
target_link_libraries(ukf_component PUBLIC ukf_core spdlog::spdlog)
ament_target_dependencies(ukf_component PUBLIC
  rclcpp rclcpp_components sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs
  tf2_ros tf2_eigen
  vortex_utils vortex_utils_ros vortex_msgs)
rclcpp_components_register_node(ukf_component PLUGIN "UKFNode" EXECUTABLE ukf_node)

install(TARGETS ukf_core ukf_component EXPORT export_ukf
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/nees_node.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_ukf HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 vortex_utils vortex_utils_ros vortex_msgs rclcpp
  sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs tf2_ros tf2_eigen spdlog)
ament_package()
