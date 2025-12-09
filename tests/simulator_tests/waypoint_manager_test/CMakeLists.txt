cmake_minimum_required(VERSION 3.8)
project(waypoint_manager_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  add_ros_isolated_launch_test(test/test_single_waypoint.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/test_service_failure.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/test_priority_service.py TIMEOUT 300)
  add_ros_isolated_launch_test(test/test_four_corner.py TIMEOUT 300)
endif()

ament_package()
