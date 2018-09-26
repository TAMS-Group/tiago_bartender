import rospy
import actionlib
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from bitbots_stackmachine.sequence_element import SequenceElement
from .actions import IdleMoveAround, WaitingToResume, MoveToCustomer, SayRepeatOrder, SayNoMenuFoundRepeat, SayOrderConfirmed, ObserveOrder, LookAtCustomer, SayPleaseOrder, LookAtMenu, MoveToBottle, LookAtBottle, MoveToPouringPosition, PourLiquid, Wait, PickUpBottle, SayDrinkFinished, LookForward, LookForCustomer, UpdateBottlePose, GetNextBottle, PlaceBottle, MoveToBottlePose, SayBottleNotFound, ExtendTorso
from tiago_bartender_msgs.msg import PourAction, PickAction, MoveToTargetAction, TakeOrderAction
from control_msgs.msg import FollowJointTrajectoryAction
from pal_interaction_msgs.msg import TtsAction
from move_base_msgs.msg import MoveBaseAction
from tiago_bartender_msgs.srv import LookAt


# @BitBots: see IMPROVE tags


class Init(AbstractDecisionElement):
    """
    Initializes the tiago bartender demo.
    Verifies ros connectivity
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)
        # set variables needed for the test
        blackboard.current_bottle = 'coke'

        self.initilized = False

    def perform(self, blackboard, reevaluate=False):
        #if not blackboard.move_action_client.wait_for_server(0.01):
        #    return self.push(WaitForRos, 'move_to_target')
        #if not blackboard.torso_action_client.wait_for_server(0.01):
        #    return self.push(WaitForRos, 'torso_controller/follow_joint_trajectory')
        #if not blackboard.tts_action_client.wait_for_server(0.01):
        #    return self.push(WaitForRos, 'tts')
        #if not blackboard.take_order_action_client.wait_for_server(0.01):
        #    return self.push(WaitForRos, 'menu/take_order')
        #if not blackboard.move_base_action_client.wait_for_server(0.01):
        #    return self.push(WaitForRos, 'move_base')
        #if not blackboard.pick_action_client.wait_for_server(0.01):
        #    return self.push(WaitForRos, 'pick')
        #if not blackboard.pour_action_client.wait_for_server(0.01):
        #    return self.push(WaitForRos, 'pour')
        #if not rospy.wait_for_service('head_controller/look_at_service', 0.01):
        #    return self.push(WaitForRos, 'head_controller/look_at_service')
        return self.push(InFrontOfRequiredBottle)

class InFrontOfRequiredBottle(AbstractDecisionElement):
    """
    Decides if the robot is in front of the bottle
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)
        blackboard.last_redoable = blackboard.PICK

    def perform(self, blackboard, reevaluate=False):
        if blackbaord.redo_requested and blackboard.last_redoable == blackboard.PICK:
            return self.push(MoveToBottle)
        elif blackboard.arrived_at_bottle:
            return self.push(BottleLocated)
        else:
            return self.push(MoveToBottle)

class BottleLocated(AbstractDecisionElement):
    """
    Locates the position of the bottle to be able to grasp it
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)
        blackboard.bottle_located = False
        blackboard.bottle_not_found = False

    def perform(self, blackboard, reevaluate=False):
        if blackboard.bottle_located:
            return self.push(BottleGrasped)
        elif blackboard.bottle_not_found:
            return self.push(SayBottleNotFound)
        else:
            return self.push_action_sequence(SequenceElement, [ExtendTorso, LookAtBottle, Wait, UpdateBottlePose], [None, 2, None])

class BottleGrasped(AbstractDecisionElement):
    """
    Grasp the bottle
    """
    def perform(self, blackboard, reevaluate=False):
        if blackboard.bottle_grasped:
            print("pick bottle succeeded")
            rospy.signal_shutdown("done")
        else:
            return self.push(PickUpBottle)

# not used yet. For later testing
class InPouringPosition(AbstractDecisionElement):
    """
    Goes to pouring position
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)
        blackboard.last_redoable = blackboard.POUR

    def perform(self, blackboard, reevaluate=False):
        if blackboard.redo_requested and blackboard.last_redoable == blackboard.POUR:
            return self.push(MoveToPouringPosition)

        if blackboard.arrived_at_pouring_position:
            # return self.push_action_sequence(SequenceElement, [ExtendTorso, PourLiquid, Wait], [None, 5])
            print("pouring succeeded")
            rospy.signal_shutdown("done")
        else:
            self.push(MoveToPouringPosition)
