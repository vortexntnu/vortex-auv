cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    test_main.cpp
    adaptive_los_test.cpp
    proportional_los_test.cpp
    integral_los_test.cpp
    vector_field_los_test.cpp

)

target_link_libraries(${TEST_BINARY_NAME}
    PRIVATE
    ${LIB_NAME}
    yaml-cpp
    Eigen3::Eigen
    spdlog::spdlog
    fmt::fmt
    GTest::GTest
)

ament_target_dependencies(${TEST_BINARY_NAME} PUBLIC Eigen3)

gtest_discover_tests(${TEST_BINARY_NAME})
