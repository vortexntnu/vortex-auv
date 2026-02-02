cmake_minimum_required(VERSION 3.8)
project(visual_egomotion)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

include_directories(include)

add_library(visual_egomotion_core
    src/visual_egomotion.cpp
)
ament_target_dependencies(visual_egomotion_core
    rclcpp
    tf2
)

target_include_directories(visual_egomotion_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(visual_egomotion_node
    src/visual_egomotion_ros.cpp
)
target_link_libraries(visual_egomotion_node visual_egomotion_core)

ament_target_dependencies(visual_egomotion_node
    rclcpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    vortex_msgs
)

install(TARGETS
    visual_egomotion_core
    visual_egomotion_node
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
