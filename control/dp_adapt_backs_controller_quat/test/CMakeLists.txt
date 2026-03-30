cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    dp_adapt_backs_controller_tests.cpp
)

target_compile_definitions(${TEST_BINARY_NAME} PRIVATE
    DRONE_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../auv_setup/config/robots/nautilus.yaml"
    CONTROLLER_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../config/adapt_params_nautilus.yaml"
)

target_link_libraries(
    ${TEST_BINARY_NAME}
    PRIVATE
    ${LIB_NAME}
    yaml-cpp
    GTest::GTest
)

ament_target_dependencies(${TEST_BINARY_NAME} PUBLIC Eigen3 tf2)

gtest_discover_tests(${TEST_BINARY_NAME})
