cmake_minimum_required(VERSION 3.5)
project(docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(DEPENDENCIES
  rclcpp
  yasmin
  yasmin_ros
  yasmin_viewer
  vortex_msgs
  std_msgs
)


add_executable(docking
  src/docking.cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(docking ${DEPENDENCIES})
install(TARGETS
  docking
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})

ament_package()
