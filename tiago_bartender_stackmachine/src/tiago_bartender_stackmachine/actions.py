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


