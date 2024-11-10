cmake_minimum_required(VERSION 3.8)
project(line_filtering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(stonefish_ros2)  # Add this to find OpenCV

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(line_filtering src/line_filtering.cpp)

ament_target_dependencies(line_filtering_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  stonefish_ros2
)

# Link against OpenCV libraries
target_link_libraries(line_filtering ${OpenCV_LIBS})

install(TARGETS
  line_filtering_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()