cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    dp_adapt_backs_controller_tests.cpp
)

target_link_libraries(
    ${TEST_BINARY_NAME}
    PRIVATE 
    ${LIB_NAME}
    GTest::GTest
)

ament_target_dependencies(${TEST_BINARY_NAME} PUBLIC Eigen3 tf2)

gtest_discover_tests(${TEST_BINARY_NAME})