cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_behavior)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib_msgs
  geometry_msgs
  tiago_bartender_navigation
  control_msgs
  actionlib
  move_base_msgs
)

add_action_files(
  DIRECTORY action
  FILES MenuOrder.action
)

add_service_files(
  FILES
  LookAt.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs actionlib_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_idle_node src/simple_idle_node.cpp)

target_link_libraries(
  simple_idle_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  simple_idle_node
  ${catkin_EXPORTED_TARGETS}
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
