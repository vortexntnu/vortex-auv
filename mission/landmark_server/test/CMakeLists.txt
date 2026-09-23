cmake_minimum_required(VERSION 3.8)
project(landmark_server_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

function(add_ros_isolated_launch_test path)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
endfunction()

add_ros_isolated_launch_test(test_intake_two_publishers.py TIMEOUT 300)
add_ros_isolated_launch_test(test_map_retention.py TIMEOUT 300)

find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(test_landmark_map
  test_class_config.cpp
  test_retained_landmarks.cpp
  test_course_frame.cpp
)
target_link_libraries(test_landmark_map PRIVATE
  landmark_server_core
  GTest::GTest
  GTest::Main
)
gtest_discover_tests(test_landmark_map)
