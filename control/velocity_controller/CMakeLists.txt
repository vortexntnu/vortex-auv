cmake_minimum_required(VERSION 3.8)
project(velocity_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CasADi REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_core REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  include
)

add_executable(velocity_controller_node
  src/velocity_controller.cpp
  src/PID_setup.cpp
  src/LQR_setup.cpp
  src/utilities.cpp
  src/ct_instantiations.cpp
)



#add_executable(test_LQR_node
#  src/LQR_test.cpp
#  src/LQR_setup.cpp
#  src/utilities.cpp
#  src/ct_instantiations.cpp
#  )
ament_target_dependencies(velocity_controller_node
  rclcpp
  std_msgs
  vortex_msgs
  geometry_msgs
  CasADi
  nav_msgs
  vortex_utils
)

target_link_libraries(velocity_controller_node Eigen3::Eigen ct_optcon ct_core)

install(TARGETS
  velocity_controller_node
  DESTINATION lib/${PROJECT_NAME}
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
