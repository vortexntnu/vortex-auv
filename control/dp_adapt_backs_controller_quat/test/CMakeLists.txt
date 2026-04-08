cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Git)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    dp_adapt_backs_controller_tests.cpp
)

execute_process(
  COMMAND ${GIT_EXECUTABLE} rev-parse --show-toplevel
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_ROOT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

target_compile_definitions(${TEST_BINARY_NAME} PRIVATE
    DRONE_YAML_PATH="${GIT_ROOT}/auv_setup/config/robots/nautilus.yaml"
    CONTROLLER_YAML_PATH="${CMAKE_SOURCE_DIR}/config/adapt_params_nautilus.yaml"
)

target_link_libraries(
    ${TEST_BINARY_NAME}
    PRIVATE
    ${LIB_NAME}
    yaml-cpp
    GTest::GTest
)

ament_target_dependencies(${TEST_BINARY_NAME} PUBLIC Eigen3)

gtest_discover_tests(${TEST_BINARY_NAME})
