cmake_minimum_required(VERSION 2.8.3)
project(behaviors)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  control_msgs
  geometry_msgs
  actionlib
  #actionlib_msgs
  
)

## System dependencies are found with CMake's conventions
##If using C++ and Boost, specify which aspects of Boost you are using as components. For example, ##if you wanted to use Boost threads, you would say:
#find_package(Boost REQUIRED COMPONENTS thread)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate actions in the 'action' folder
#add_action_files(
#   DIRECTORY action FILES FollowJointTrajectory.action
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   sensor_msgs 
#   geometry_msgs
#   control_msgs
#   actionlib_msgs
#)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs control_msgs geometry_msgs actionlib_msgs
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fetch_turn_head_right src/fetch_turn_head_right.cpp)
add_executable(fetch_look_around src/fetch_look_around.cpp)
add_executable(fetch_back_up src/fetch_back_up.cpp)
add_executable(fetch_back_up_turn_away_left src/fetch_back_up_turn_away_left.cpp)
add_executable(fetch_turn_away_left src/fetch_turn_away_left.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(fetch_turn_head_right ${catkin_LIBRARIES})
target_link_libraries(fetch_look_around ${catkin_LIBRARIES})
target_link_libraries(fetch_back_up ${catkin_LIBRARIES})
target_link_libraries(fetch_back_up_turn_away_left ${catkin_LIBRARIES})
target_link_libraries(fetch_turn_away_left ${catkin_LIBRARIES})
