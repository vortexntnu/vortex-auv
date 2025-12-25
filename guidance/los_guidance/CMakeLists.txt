cmake_minimum_required(VERSION 3.8)
project(los_guidance)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(include)

set(LIB_NAME los_guidance_lib)
add_library(${LIB_NAME} SHARED
  src/lib/proportional_los.cpp
  src/lib/integral_los.cpp
  src/lib/adaptive_los.cpp
  src/los_guidance_ros.cpp
)

ament_target_dependencies(${LIB_NAME}
  rclcpp
  rclcpp_action
  geometry_msgs
  vortex_msgs
  vortex_utils_ros
  Eigen3
  spdlog
  fmt
)

add_executable(los_guidance_node
  src/los_guidance_node.cpp
)

target_link_libraries(los_guidance_node
  ${LIB_NAME}
  yaml-cpp
)

install(TARGETS
  ${LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  los_guidance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  include(CTest)
  add_subdirectory(test)
endif()

ament_package()
