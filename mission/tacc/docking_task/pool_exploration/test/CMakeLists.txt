find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_pool_exploration
  test_pool_exploration.cpp
)

target_include_directories(test_pool_exploration PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

ament_target_dependencies(test_pool_exploration
  rclcpp
  nav_msgs
  vortex_msgs
  Eigen3
)

target_link_libraries(test_pool_exploration
  ${PROJECT_NAME}_lib
)