cmake_minimum_required(VERSION 3.8)
project(line_filtering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(line_filtering_node src/line_filtering_ros.cpp src/line_filtering_node.cpp)

ament_target_dependencies(line_filtering_node
  rclcpp
  sensor_msgs
  cv_bridge
  std_msgs
  geometry_msgs
)

# Link against OpenCV libraries
target_link_libraries(line_filtering_node ${OpenCV_LIBS})

install(TARGETS
  line_filtering_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()