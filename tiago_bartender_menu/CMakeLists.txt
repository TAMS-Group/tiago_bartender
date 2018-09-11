cmake_minimum_required(VERSION 2.9.3)
project(tiago_bartender_menu)

set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE Release)

add_compile_options($<$<CONFIG:Release>:-O3>)
add_compile_options($<$<CONFIG:Release>:-ffast-math>)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf_conversions
  roscpp
  std_srvs
  visualization_msgs
  interactive_markers
  image_transport
  sensor_msgs
  cv_bridge
  tiago_bartender_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf
  tf_conversions
  sensor_msgs
  cv_bridge
  actionlib_msgs
  image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS program_options)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_executable(tiago_bartender_menu src/tiago_bartender_menu.cpp)
add_dependencies(tiago_bartender_menu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tiago_bartender_menu yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
