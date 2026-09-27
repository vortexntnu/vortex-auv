cmake_minimum_required(VERSION 3.8)
project(landmark_server_test)

find_package(ament_cmake REQUIRED)
find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(test_landmark_map
  test_class_config.cpp
  test_retained_landmarks.cpp
  test_course_frame.cpp
  test_map_rules.cpp
)
target_link_libraries(test_landmark_map PRIVATE
  landmark_server_core
  GTest::GTest
  GTest::Main
)
gtest_discover_tests(test_landmark_map)

add_executable(test_landmark_graph test_landmark_graph.cpp)
target_link_libraries(test_landmark_graph PRIVATE
  landmark_server_graph
  GTest::GTest
  GTest::Main
)
gtest_discover_tests(test_landmark_graph)
