cmake_minimum_required(VERSION 3.14)
project(pipeline_test_simulation)

set(pipeline_test_simulation_MAJOR_VERSION 0)
set(pipeline_test_simulation_MINOR_VERSION 0)
set(pipeline_test_simulation_PATCH_VERSION 0)
set(pipeline_test_simulation_VERSION
  ${pipeline_test_simulation_MAJOR_VERSION}.${pipeline_test_simulation_MINOR_VERSION}.${pipeline_test_simulation_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pipeline_test_interfaces REQUIRED)
