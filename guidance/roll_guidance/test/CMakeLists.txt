cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
include(GoogleTest)

# Unit tests: T01–T12
set(TEST_BINARY_NAME ${PROJECT_NAME}_test)

add_executable(${TEST_BINARY_NAME}
    test_roll_guidance.cpp
)

target_link_libraries(${TEST_BINARY_NAME}
    PRIVATE
    ${LIB_NAME}
    GTest::GTest
)

ament_target_dependencies(${TEST_BINARY_NAME}
    PUBLIC
    Eigen3
)

gtest_discover_tests(${TEST_BINARY_NAME})


# Integration tests: T13–T27
foreach(test_name
    test_gate_crossing
    test_roll_guidance_node
)
    add_executable(${test_name}
        integration/${test_name}.cpp
        integration/test_main.cpp
    )

    target_include_directories(${test_name}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/integration
    )

    target_link_libraries(${test_name}
        PRIVATE
        ${LIB_NAME}
        GTest::GTest
    )

    ament_target_dependencies(${test_name}
        PUBLIC
        rclcpp
        rclcpp_action
        geometry_msgs
        vortex_msgs
        Eigen3
    )

    gtest_discover_tests(${test_name})
endforeach()