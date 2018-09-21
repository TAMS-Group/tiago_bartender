from __future__ import print_function

import rospy
import actionlib
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
from tiago_bartender_msgs.msg import PourAction, PickAction

class IdleMoveAround(AbstractActionElement):
    """
    move to random pose behind the counter
    """
    def perform(self, blackboard, reevaluate=False):
        print("IdleMoveAround")
        rospy.sleep(0.1)
        return self.pop()
        #TODO actually do something


class WaitingToResume(AbstractActionElement):
    """
    This action doesn't do anything and is only put on the stack for visualization purposes
    """
    def perform(self, blackboard, reevaluate=False):
        rospy.loginfo_throttle(10, "I'm currently paused. Please show pause card to unpause me.")


class MoveToCustomer(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        print("MoveToCustomer")
        blackboard.arrived_at_customer = True
        #TODO call action

class MoveToBottle(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        print("MoveToBottle")
        blackboard.arrived_at_bottle = True
        #TODO call action

class MoveToPouringPosition(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        print("MoveToPouringPosition")
        blackboard.arrived_at_pouring_position = True
        #TODO call action

class AbstractLookAt(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        target = self.target()
        print("Looking at " + target)
        #TODO call look at service

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
        rospy.sleep(0.1)
        return self.pop()
        #TODO call action speach service or espeak or what ever

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

class SayDrinkFinished(AbstractSay):
    def text(self):
        return "Your drink is finished"



class ObserveOrder(AbstractActionElement):
    def __init__(self, blackboard, _):
        self.first_try = True

    def perform(self, blackboard, reevaluate=False):
        blackboard.no_menu_found = False
        #TODO call take order action
        if self.first_try:
            print("ObserveOrder - fail")
            blackboard.no_menu_found = True
            self.first_try= False
        else:
            print("ObserveOrder - fail")
            blackboard.recipe
        # TODO: set blackboard.recipe

   
class PickUpBottle(AbstractActionElement):
    """
    Calls the pick action
    """
    def __init__(self, blackboard, _):
        super(PickUpBottle, self).__init__(blackboard)        
        goal = Pick()
        #TODO specify goal
        blackboard.pick_action_client.send_goal(goal)

    def perform(self, blackboard, reevaluate=False):
        state = blackboard.pick_action_client.get_state()
        # wait till action is completed
        if state == 3:
            #TODO maybe do something with the result
            self.pop()

class PourLiquid(AbstractActionElement):
    """
    Calls the pouring action
    """
    def __init__(self, blackboard, _):
        super(PourLiquid, self).__init__(blackboard)        
        goal = Pour()
        #TODO specify goal
        blackboard.pour_action_client.send_goal(goal)

    def perform(self, blackboard, reevaluate=False):
        state = blackboard.pour_action_client.get_state()
        # wait till action is completed
        if state == 3:
            #TODO maybe do something with the result
            self.pop()
            


class Wait(AbstractActionElement):
    """
    Waits the amount of seconds given on init and pops
    """
    def __init__(self, blackboard, args=10):
        super(Wait, self).__init__(blackboard)
        self.resume_time = rospy.get_time() + args

    def perform(self, connector, reevaluate=False):       
        if self.resume_time < rospy.get_time():
            self.pop()        
