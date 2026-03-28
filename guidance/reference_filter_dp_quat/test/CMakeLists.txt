cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    test_reference_filter.cpp
)

target_link_libraries(
    ${TEST_BINARY_NAME}
    PRIVATE
    ${CORE_LIB_NAME}
    GTest::GTest
)

ament_target_dependencies(${TEST_BINARY_NAME} PUBLIC Eigen3)

gtest_discover_tests(${TEST_BINARY_NAME})

# Waypoint follower tests
set(TEST_WAYPOINT_FOLLOWER ${PROJECT_NAME}_test_waypoint_follower)
add_executable(
    ${TEST_WAYPOINT_FOLLOWER}
    test_waypoint_follower.cpp
)

target_link_libraries(
    ${TEST_WAYPOINT_FOLLOWER}
    PRIVATE
    ${CORE_LIB_NAME}
    GTest::GTest
)

ament_target_dependencies(${TEST_WAYPOINT_FOLLOWER} PUBLIC Eigen3 vortex_utils)

gtest_discover_tests(${TEST_WAYPOINT_FOLLOWER})
