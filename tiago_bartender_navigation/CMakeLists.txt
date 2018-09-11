cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  actionlib
  actionlib_msgs
  std_msgs
  geometry_msgs
  move_base_msgs
  moveit_ros_planning_interface
  tiago_bartender_msgs
  tf
)

add_action_files(
  DIRECTORY action
  FILES
  MoveToTarget.action
  FindClosestTarget.action
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(move_to_target_node src/move_to_target_node.cpp)

add_dependencies(move_to_target_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(move_to_target_node
  ${catkin_LIBRARIES}
)

