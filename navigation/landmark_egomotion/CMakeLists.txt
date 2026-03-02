cmake_minimum_required(VERSION 3.8)
project(landmark_egomotion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eskf REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT DEFINED EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

include_directories(${EIGEN3_INCLUDE_DIR})

set(LIB_NAME "${PROJECT_NAME}_lib")

add_library(${LIB_NAME} SHARED
  src/lib/landmark_egomotion.cpp
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${LIB_NAME} PUBLIC
  eskf::eskf
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  Eigen3
)

ament_export_targets(export_${LIB_NAME})
ament_export_include_directories(include)

install(TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
