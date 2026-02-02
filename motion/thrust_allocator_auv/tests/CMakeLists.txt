cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    thrust_allocator_tests.cpp
)

target_compile_definitions(thrust_allocator_auv_test PRIVATE
  YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../auv_setup/config/robots/orca.yaml"
)

target_link_libraries(
    ${TEST_BINARY_NAME}
    PRIVATE
    ${LIB_NAME}
    yaml-cpp
    GTest::GTest
    spdlog::spdlog
)

ament_target_dependencies(${TEST_BINARY_NAME} PUBLIC Eigen3 tf2 vortex_utils)

gtest_discover_tests(${TEST_BINARY_NAME})
