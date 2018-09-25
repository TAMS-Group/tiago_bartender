cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_world)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  moveit_ros_planning_interface
  tf
  tiago_bartender_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES find_user_exekutor
#  CATKIN_DEPENDS action_exekutor roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dummy_planning_scene_init src/dummy_planning_scene_init.cpp)
target_link_libraries(dummy_planning_scene_init ${catkin_LIBRARIES})
add_dependencies(dummy_planning_scene_init ${catkin_EXPORTED_TARGETS})

add_executable(planning_scene_init src/planning_scene_init.cpp)
target_link_libraries(planning_scene_init ${catkin_LIBRARIES})
add_dependencies(planning_scene_init ${catkin_EXPORTED_TARGETS})
