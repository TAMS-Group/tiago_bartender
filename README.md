# tiago_bartender

Launch files, messages, high-level coordination, and other modules for our TIAGo-based bartender demo, 
demonstrated at the Mobile Manipulation Hackathon during IROS-2018 in Madrid, based on ROS kinetic.

All custom modules ran on an external laptop mounted to the robot,
in some cases overriding modules on the robot, especially MoveIt.

See tiago_bartender_bringup/config/tams-runtime-changes.sh
for changes required on the robot's computer.

# rosinstall files

The system was setup in two workspaces on top of /opt/ros:
- ros-system, providing modules we overlayed or non-standard modules not available in debs
- ros, with everything we implemented for the demonstration

The corresponding .rosinstall files for both workspaces with build instructions
can be found in rosinstalls/ .

# Modules in this repository

## tiago_bartender_bringup

launch files for the whole demo

The top-level launch file (with comments) is
tiago_bartender_bringup/launch/tiago_bar_demo.launch

## tiago_bartender_msgs

demo-specific ROS interface definitions

## tiago_bartender_behavior

provides LookAt action

## tiago_bartender_menu

detect menu card and orders from pointing gestures

## tiago_bartender_navigation

provide MoveTo action for intelligent navigation towards targets
in the partially-known planning scene and bar scenario

## tiago_bartender_stackmachine

high-level control and coordination used for the demo.

This is based on the BitBots stack-machine paradigm.

## tiago_bartender_state_machine

Earlier version of the coordination module based on a simple
state-machine concept implemented in C++.


## tiago_bartender_world

auxiliary files for Gazebo scene description
and planning scene maintainance
