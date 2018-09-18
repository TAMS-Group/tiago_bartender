cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_stackmachine)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  bitbots_stackmachine
  rospy
)

catkin_python_setup()

catkin_package()
