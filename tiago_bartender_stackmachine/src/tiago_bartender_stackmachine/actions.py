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



class AbstractSay(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        text = self.text()
        print("Saying '" + text + "'")

    def text(self):
        raise NotImplementedError



class SayRepeatOrder(AbstractSay):
    def text(self):
        return "Oh, so you want something different?"

