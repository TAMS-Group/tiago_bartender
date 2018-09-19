from __future__ import print_function

import rospy
from bitbots_stackmachine.abstract_action_element import AbstractActionElement


class IdleMoveAround(AbstractActionElement):
    """
    move to random pose behind the counter
    """

    def __init__(self, blackboard, _):
        pass

    def perform(self, blackboard, reevaluate=False):
        print("IdleMoveAround")



class Noop(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        print("Noop")



class MoveToCustomer(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        print("MoveToCustomer")
        blackboard.arrived_at_customer = True



class AbstractSay(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        text = self.text()
        print("Saying '" + text + "'")

    def text(self):
        raise NotImplementedError



class SayRepeatOrder(AbstractSay):
    def text(self):
        return "Oh, so you want something different?"


class ObserveOrder(AbstractActionElement):
    def __init__(self, blackboard, _):
        self.first_try = True

    def perform(self, blackboard, reevaluate=False):
        blackboard.no_menu_found = False

        if self.first_try:
            print("ObserveOrder - fail")
            blackboard.no_menu_found = True
            self.first_try= False
        else:
            print("ObserveOrder - fail")
            blackboard.recipe
        # TODO: set blackboard.recipe
