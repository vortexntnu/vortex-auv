cmake_minimum_required(VERSION 3.8)
project(line_filtering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(vortex_filtering REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(line_filtering_node src/line_filtering_ros.cpp src/track_manager.cpp src/line_filtering_node.cpp)

# Link against OpenCV libraries
target_link_libraries(line_filtering_node ${OpenCV_LIBS} Eigen3::Eigen)

ament_target_dependencies(line_filtering_node
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  std_msgs
  geometry_msgs
  vortex_filtering
  visualization_msgs
)


install(TARGETS
  line_filtering_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
