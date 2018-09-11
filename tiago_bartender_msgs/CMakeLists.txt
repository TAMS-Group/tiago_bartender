cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_msgs)

set(MSG_DEPS
	actionlib_msgs
	geometry_msgs
	std_msgs
)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	message_generation
	${MSG_DEPS}
)

## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEPS
## * In this file (CMakeLists.txt):
##   * add every package in MSG_DEPS to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add every package in MSG_DEPS to generate_messages(DEPENDENCIES ...)

add_message_files(
	FILES
	ManipulationResult.msg
)

add_service_files(FILES
	LookAt.srv
)

add_action_files(
	FILES
		Pick.action
		Pour.action
		Place.action
		TakeOrder.action
		MoveToTarget.action
		FindClosestTarget.action
		BartenderSpeech.action
)

generate_messages(
DEPENDENCIES
	${MSG_DEPS}
)

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
