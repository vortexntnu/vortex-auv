if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(test_landmark_server
  test_landmark_server.cpp
)

target_include_directories(test_landmark_server PRIVATE
  ${CMAKE_SOURCE_DIR}/../include
)

ament_target_dependencies(test_landmark_server rclcpp)
