cmake_minimum_required(VERSION 3.8)
project(pipeline_cpp)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined." )
endif()
if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  message(STATUS "Build for ROS2 Foxy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DFOXY")
elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(STATUS "Build for ROS2 Humble")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHUMBLE")
elseif("$ENV{ROS_DISTRO}" STREQUAL "iron")
  message(STATUS "Build for ROS2 Iron")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIRON")
elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  message(STATUS "Build for ROS2 Jazzy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DJAZZY")
elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  message(STATUS "Build for ROS2 Rolling")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROLLING")
else()
  message(FATAL_ERROR "Unsupported ROS Distribution: " "$ENV{ROS_DISTRO}")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(DEPENDENCIES
  rclcpp
  yasmin
  yasmin_ros
  yasmin_viewer
  nav_msgs
  vortex_msgs
  rclcpp
  std_msgs
)


# action client demo
add_executable(pipeline_demo_cpp
  src/pipeline_demo_cpp.cpp
)

set(SOURCES
  src/pipeline_demo_cpp.cpp
)
ament_target_dependencies(pipeline_demo_cpp ${DEPENDENCIES})
install(TARGETS
  pipeline_demo_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})

ament_package()
