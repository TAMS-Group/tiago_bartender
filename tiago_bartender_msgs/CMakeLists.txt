cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_msgs)

set(MSG_DEPS
	geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	${MSG_DEPS}
)

## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEPS
## * In this file (CMakeLists.txt):
##   * add every package in MSG_DEPS to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add every package in MSG_DEPS to generate_messages(DEPENDENCIES ...)

#add_message_files(
#	FILES
#	Message1.msg
#)

add_service_files(FILES
	LookAt.srv
)

#add_action_files(
#	FILES
#		Action1.action
#		Action2.action
#)

generate_messages(
DEPENDENCIES
	${MSG_DEPS}
)

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
