cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(test_landmark_targets
  test_resolve_target.cpp
  test_landmark_target.cpp
  test_geometry.cpp
)
target_link_libraries(test_landmark_targets PRIVATE
  ${CORE_LIB_NAME}
  GTest::GTest
  GTest::Main
)
ament_target_dependencies(test_landmark_targets PUBLIC Eigen3 vortex_utils)
gtest_discover_tests(test_landmark_targets)
