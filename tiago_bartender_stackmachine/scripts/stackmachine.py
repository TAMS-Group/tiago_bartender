#!/usr/bin/env python

import rospy

from bitbots_stackmachine.stack_machine import StackMachine
from tiago_bartender_stackmachine.decisions import Init
from tiago_bartender_stackmachine.blackboard import Blackboard

class TiagoBartender:
    def __init__(self):
        rospy.init_node('tiago_bartender_stackmachine')

        self.blackboard = Blackboard()
        self.stackmachine = StackMachine(self.blackboard, "controller")
	self.stackmachine.set_start_element(Init)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.stackmachine.update()


def main():
    bartender = TiagoBartender()
    bartender.run()


if __name__ == '__main__':
    main()
