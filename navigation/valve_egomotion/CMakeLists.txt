cmake_minimum_required(VERSION 3.8)
project(valve_egomotion)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(valve_egomotion_core
    src/valve_egomotion.cpp
)
ament_target_dependencies(valve_egomotion_core
    rclcpp
    tf2
)

target_include_directories(valve_egomotion_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(valve_egomotion_node
    src/valve_egomotion_ros.cpp
)
target_link_libraries(valve_egomotion_node valve_egomotion_core)

ament_target_dependencies(valve_egomotion_node
    rclcpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

install(TARGETS
    valve_egomotion_core
    valve_egomotion_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
