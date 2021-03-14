cmake_minimum_required(VERSION 3.5.1)

project(eskf
        VERSION 0.1
        LANGUAGES CXX)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)

add_subdirectory(src)
#add_subdirectory(apps/test)

find_package(roscpp QUIET)
if (roscpp_FOUND)
    # TODO: remove unused components
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        tf
)

# TODO: remove unused dependencies
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES 
        CATKIN_DEPENDS geometry_msgs roscpp message_runtime sensor_msgs tf 
        # DEPENDS eigen system_lib
)

include_directories(
	PRIVATE
        apps/ros
	PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(eskf apps/ros/ros_node.cpp)

target_link_libraries(eskf PRIVATE eskf_lib ${catkin_LIBRARIES})

add_executable(eskf_test apps/test/main.cpp)
target_link_libraries(eskf_test PRIVATE eskf_lib)

endif ()
