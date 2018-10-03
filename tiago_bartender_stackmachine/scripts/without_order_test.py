#!/usr/bin/env python

import rospy

from bitbots_stackmachine.stack_machine import StackMachine
from tiago_bartender_stackmachine.without_order_decisions import Init
from tiago_bartender_stackmachine.blackboard import Blackboard

class TiagoBartender:
    def __init__(self):
        rospy.init_node('tiago_bartender_stackmachine')

        self.blackboard = Blackboard()
        self.stackmachine = StackMachine(self.blackboard, "/debug_stackmachine")
        rospy.sleep(1) # needed to prevent race condition with initilizing the publisher of the stack machine. without the vizualization will not have data of the init
    	self.stackmachine.set_start_element(Init)
        #TODO self.pause_card_subscriber(..., self.pause_card_cb)
        #TODO self.redo_card_subscriber(..., self.redo_card_cb)

        self.run()        

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.stackmachine.update()
        print("exit stackmachine")

    def pause_card_cb(self, msg):
        self.blackboard.was_pause_card_shown = True

    def redo_card_cb(self, msg):
        self.blackboard.redo_requested = True


def main():
    bartender = TiagoBartender()
    bartender.run()


if __name__ == '__main__':
    main()
