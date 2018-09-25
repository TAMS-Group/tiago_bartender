cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_behavior)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib_msgs
  geometry_msgs
  tiago_bartender_msgs
  control_msgs
  actionlib
  move_base_msgs
  pal_common_msgs
  moveit_ros_planning_interface
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs actionlib_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(look_at_node src/look_at_node.cpp)

target_link_libraries(
  look_at_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  look_at_node
  ${catkin_EXPORTED_TARGETS}
)
