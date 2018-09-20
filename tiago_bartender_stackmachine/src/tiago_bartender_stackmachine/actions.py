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

class MoveToBottle(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        print("MoveToBottle")
        blackboard.arrived_at_bottle = True


class MoveToPouringPosition(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        print("MoveToPouringPosition")
        blackboard.arrived_at_pouring_position = True

class AbstractLookAt(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        target = self.target()
        print("Looking at " + target)

    def target(self):
        raise NotImplementedError


class LookAtCustomer(AbstractLookAt):
    def target(self):
        return "Customer"

class LookAtBottle(AbstractLookAt):
    def target(self):
        return "Bottle"

class LookAtMenu(AbstractLookAt):
    def target(self):
        return "Menu"


class AbstractSay(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        text = self.text()
        print("Saying '" + text + "'")

    def text(self):
        raise NotImplementedError

class SayRepeatOrder(AbstractSay):
    def text(self):
        return "Oh, so you want something different?"

class SayNoMenuFoundRepeat(AbstractSay):
    def text(self):
        return "No menue found, please repeat"

class SayPleaseOrder(AbstractSay):
    def text(self):
        return "Please Order"

class SayOrderConfirmed(AbstractSay):
    def text(self):
        return "Order confirmed"



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

class GraspBottle(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        print("gasp Bottle")

class PourLiquid(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        print("pour liquid")

class Wait(AbstractActionElement):
    """
    Just waits a moment
    """
    def perform(self, blackboard, reevaluate=False):
        print("wait")