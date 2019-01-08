cmake_minimum_required(VERSION 2.8.3)
project(manta_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED) # this is only required to get gazebo's directories

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_descriptions
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Install python scripts
catkin_install_python(PROGRAMS scripts/spawn_model
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch Media worlds models world_models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
