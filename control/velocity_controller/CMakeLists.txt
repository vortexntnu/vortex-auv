cmake_minimum_required(VERSION 3.8)
project(velocity_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_core REQUIRED)
find_package(casadi REQUIRED)

include_directories(
  include
)


set(LIB_NAME velocity_controller_component)
add_library(${LIB_NAME} SHARED
  src/velocity_controller.cpp
  src/PID_setup.cpp
  src/LQR_setup.cpp
  src/utilities.cpp
  src/ct_instantiations.cpp
)
ament_target_dependencies(${LIB_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  vortex_msgs
  geometry_msgs
  nav_msgs
  vortex_utils
)
target_link_libraries(${LIB_NAME} Eigen3::Eigen casadi::casadi ct_optcon ct_core)
install(TARGETS
  ${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_package()
