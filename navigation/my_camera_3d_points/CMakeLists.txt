cmake_minimum_required(VERSION 3.8)
project(my_camera_3d_points)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)  # Add this to find OpenCV

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(camera_3d_points_node src/camera_3d_points_node.cpp)

ament_target_dependencies(camera_3d_points_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

# Link against OpenCV libraries
target_link_libraries(camera_3d_points_node ${OpenCV_LIBS})

install(TARGETS
  camera_3d_points_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()