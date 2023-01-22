cmake_minimum_required(VERSION 3.0.2)
project(actionlib_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
)



## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs #   std_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES actionlib_tutorials
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


# Server 
add_executable(
  ${PROJECT_NAME}_server src/fibonacci_server.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)


# Client
add_executable(
  ${PROJECT_NAME}_client src/fibonacci_client.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}_client
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)




# add_executable(
#   ${PROJECT_NAME}_server src/fibonacci_server.cpp
#   ${PROJECT_NAME}_client src/fibonacci_client.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(
#   ${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# ## Add cmake target dependencies of the executable
# ## same as for the library above
# add_dependencies(
#   ${PROJECT_NAME}_node 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#   ${catkin_EXPORTED_TARGETS}
#   )




## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
