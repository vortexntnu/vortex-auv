cmake_minimum_required(VERSION 3.5)
project(docking_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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



add_executable(docking
  src/docking.cpp
)
add_executable(docking_node
  src/docking_node.cpp
)

set(SOURCES
  src/docking.cpp
  src/docking_node.cpp
)
ament_target_dependencies(docking ${DEPENDENCIES})
ament_target_dependencies(docking_node ${DEPENDENCIES})
install(TARGETS
  docking
  docking_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})

ament_package()
