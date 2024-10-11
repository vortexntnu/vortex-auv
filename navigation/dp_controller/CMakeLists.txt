cmake_minimum_required(VERSION 3.8)
project(dp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executable
add_executable(${PROJECT_NAME}_node src/main.cpp)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node rclcpp Eigen3 std_msgs nav_msgs geometry_msgs)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
