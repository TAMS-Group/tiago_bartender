project(tiago_bartender_menu)

set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE Release)

add_compile_options($<$<CONFIG:Release>:-O3>)
add_compile_options($<$<CONFIG:Release>:-ffast-math>)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  tf
  tf_conversions
  roscpp
  std_srvs
  moveit_ros_planning_interface
  visualization_msgs
  interactive_markers
  moveit_ros_robot_interaction
  sensor_msgs
  cv_bridge
  actionlib_msgs
)

add_action_files(
  DIRECTORY action
  FILES TakeOrder.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning
  roscpp
  tf
  tf_conversions
  sensor_msgs
  cv_bridge
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS program_options)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_executable(tiago_bartender_menu src/tiago_bartender_menu.cpp)
add_dependencies(tiago_bartender_menu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tiago_bartender_menu yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
