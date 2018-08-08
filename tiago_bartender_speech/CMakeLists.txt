cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_speech)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  actionlib
  actionlib_msgs
  sound_play
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES
  BartenderSpeech.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roslib actionlib_msgs
#  INCLUDE_DIRS include
#  LIBRARIES tiago_bartender_speech
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(play_file_node src/play_file_node.cpp)

target_link_libraries(
  play_file_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  play_file_node
  ${catkin_EXPORTED_TARGETS}
)
