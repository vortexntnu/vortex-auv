cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake REQUIRED)
find_package(GTest REQUIRED)
find_package(vortex_filtering REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    test_pose_filtering.cpp
)

target_link_libraries(
  ${TEST_BINARY_NAME}
  ${PROJECT_NAME}
  GTest::GTest
  GTest::gtest_main
)

ament_target_dependencies(
  ${TEST_BINARY_NAME}
  vortex_filtering
  vortex_utils
  Eigen3
  tf2
)

gtest_discover_tests(${TEST_BINARY_NAME})
