cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_state_machine)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  tiago_bartender_behavior
  tiago_bartender_msgs
  tiago_bartender_navigation
  tiago_bartender_speech
  tiago_bartender_menu
  move_base_msgs
  tiago_bartender_msgs
  moveit_ros_planning_interface
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_machine_node src/state_machine_node.cpp)
add_dependencies(state_machine_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine_node
  ${catkin_LIBRARIES}
)
