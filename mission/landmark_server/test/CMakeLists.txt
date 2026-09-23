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
