cmake_minimum_required(VERSION 3.5)
project(publish_docking_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(DEPENDENCIES
  rclcpp
  yasmin_msgs
  std_msgs
)


add_executable(publish_docking_state
  src/publish_docking_state.cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_target_dependencies(publish_docking_state ${DEPENDENCIES})
install(TARGETS
  publish_docking_state
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})

ament_package()
