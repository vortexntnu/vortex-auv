cmake_minimum_required(VERSION 3.8)
project(valve_egomotion)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

include_directories(include)

add_executable(valve_egomotion
    src/valve_egomotion.cpp
)

ament_target_dependencies(valve_egomotion
    rclcpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    vortex_msgs
)

install(TARGETS valve_egomotion
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

ament_package()
