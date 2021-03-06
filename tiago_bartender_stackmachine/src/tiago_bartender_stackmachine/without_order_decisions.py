import rospy
import actionlib
import copy
import sys
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from bitbots_stackmachine.sequence_element import SequenceElement
from .actions import IdleMoveAround, WaitingToResume, MoveToCustomer, SayRepeatOrder, SayNoMenuFoundRepeat, SayOrderConfirmed, ObserveOrder, LookAtCustomer, SayPleaseOrder, LookAtMenu, MoveToBottle, LookAtBottle, MoveToPouringPosition, PourLiquid, Wait, PickUpBottle, SayDrinkFinished, LookForward, LookForCustomer, LookDefault, UpdateBottlePose, GetNextBottle, PlaceBottle, MoveToBottlePose, SayBottleNotFound, WaitForRos, ExtendTorso, SayGlassNotFound, UpdateGlassPose, SearchBottleLeft, SearchBottleRight, SayAcid, LookAtGlass, LookAtPlacePose, HomePose, SayDrinkNotFound, UpdateCustomerPose
from tiago_bartender_msgs.msg import PourAction, PickAction, MoveToTargetAction, TakeOrderAction
from control_msgs.msg import FollowJointTrajectoryAction
from pal_interaction_msgs.msg import TtsAction
from move_base_msgs.msg import MoveBaseAction
from tiago_bartender_msgs.srv import LookAt
import moveit_commander


# @BitBots: see IMPROVE tags


class Init(AbstractDecisionElement):
    """
    Initializes the tiago bartender demo.
    Verifies ros connectivity
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)        
        self.initilized = False
        self.first_iteration = True

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration:
            if len(sys.argv) > 1:
                blackboard.current_drink = sys.argv[1]
            else:
                blackboard.current_drink = 'water'
            blackboard.recipe = copy.deepcopy(blackboard.current_drink)
            self.first_iteration = False
            return self.push(SayOrderConfirmed)

        if not blackboard.move_action_client.wait_for_server(rospy.Duration(0.01)):
            return self.push(WaitForRos, 'move_to_target')
        if not blackboard.torso_action_client.wait_for_server(rospy.Duration(0.01)):
            return self.push(WaitForRos, 'torso_controller/follow_joint_trajectory')
        if not blackboard.tts_action_client.wait_for_server(rospy.Duration(0.01)):
            return self.push(WaitForRos, 'tts')
        if not blackboard.take_order_action_client.wait_for_server(rospy.Duration(0.01)):
            return self.push(WaitForRos, 'menu/take_order')
        if not blackboard.move_base_action_client.wait_for_server(rospy.Duration(0.01)):
            return self.push(WaitForRos, 'move_base')
        if not blackboard.pick_action_client.wait_for_server(rospy.Duration(0.01)):
            return self.push(WaitForRos, 'pick')
        #if not blackboard.pour_action_client.wait_for_server(rospy.Duration(0.01)):
        #    return self.push(WaitForRos, 'pour')
        if not blackboard.detect_bottles_action_client.wait_for_server(rospy.Duration(0.01)):
            return self.push(WaitForRos, 'detect_bottles_action')
        if not blackboard.detect_glass_action_client.wait_for_server(rospy.Duration(0.01)):
            return self.push(WaitForRos, 'detect_glass_action')
	try:
            rospy.wait_for_service('head_controller/look_at_service', 0.1)
	except rospy.ROSException:
            return self.push(WaitForRos, '/head_controller/look_at_service')

        return self.push(Paused)

# TODO: refactor as AbstractPausableAction
class Paused(AbstractDecisionElement):
    """
    Check whether the system should is paused.
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)        
        self.saved_stack = None

    def get_reevaluate(self):
        return True

    def perform(self, blackboard, reevaluate=False):
        if blackboard.is_paused:
            if self.saved_stack is None:
                # IMPROVE: the stack should *not* be called _behaviour, especially not in BE
                self.saved_stack = self._behaviour
            return self.push(WaitingToResume)
        else:
            if self.saved_stack:
                # see if we already saved a stack (meaning we're currently paused)
                self._behaviour = self.saved_stack
                self.saved_stack = None
                return
            else:
                return self.push(MakeCocktail)

class MakeCocktail(AbstractDecisionElement):
    """
    Make a cocktail out of multiple ingrediances
    """
    def perform(self, blackboard, reevaluate=False):
        if blackboard.get_next_bottle and len(blackboard.recipe)>0:
            # add next liquid
            return self.push(GetNextBottle)
        if blackboard.got_next_bottle:
            return self.push(PutBottleBack)
        else:
            # we're finished
            return self.push(DrinkFinished)

    def get_reevaluate(self):
        return True

class DrinkFinished(AbstractDecisionElement):
    """
    The Drink is finished. Tell it to the costumer and clean up
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)
        self.first = True

    def perform(self, blackboard, reevaluate=False):
        if self.first:
            self.first = False
            return self.push_action_sequence(SequenceElement, [LookForward, Wait, SayDrinkFinished], [None, 2, None])
        else:
            return self.push(PutLastBottleBack)

class PutLastBottleBack(AbstractDecisionElement):
    """
    Put away last bottle and go back to init via interrupt.
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)
        blackboard.arrived_at_bottle = False

    def perform(self, blackboard, reevaluate=False):
        if not blackboard.arrived_at_bottle:
            first_iteration = False
            return self.push_action_sequence(SequenceElement, [LookDefault, MoveToBottlePose], [None, None])
        elif blackboard.bottle_grasped:
            return self.push_action_sequence(SequenceElement, [ExtendTorso, LookAtPlacePose, Wait, UpdateBottlePose, PlaceBottle], [None, None, 4, None, None])
        else:
            rospy.signal_shutdown('done')
            # resetting variables in blackboard and going back to HasCustomer
            #blackboard.reset()

class PutBottleBack(AbstractDecisionElement):
    """
    Put away bottle.
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)
        blackboard.arrived_at_bottle = False

    def perform(self, blackboard, reevaluate=False):
        if not blackboard.arrived_at_bottle and blackboard.bottle_grasped:
            return self.push_action_sequence(SequenceElement, [LookDefault, MoveToBottlePose], [None, None])
        elif blackboard.bottle_grasped:
            return self.push_action_sequence(SequenceElement, [ExtendTorso, LookAtPlacePose, Wait, UpdateBottlePose, PlaceBottle], [None, None, 4, None, None])
        else:
            return self.push(InFrontOfRequiredBottle)

class InFrontOfRequiredBottle(AbstractDecisionElement):
    """
    Decides if the robot is in front of the bottle
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)        
        blackboard.last_redoable = blackboard.PICK
        blackboard.arrived_at_bottle = False

    def perform(self, blackboard, reevaluate=False):
        if blackboard.redo_requested and blackboard.last_redoable == blackboard.PICK:
            blackboard.redo_requested = False
            #TODO maybe go back to init position
            return self.push(MoveToBottle)
        elif blackboard.arrived_at_bottle:
            return self.push(BottleLocated)
        else:
            return self.push(MoveToBottle)

    def get_reevaluate(self):
        return True

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
            return self.push_action_sequence(SequenceElement, [SearchBottleLeft, Wait, SearchBottleRight, SayBottleNotFound], [None, 2, None, None])
        else:
            return self.push_action_sequence(SequenceElement, [ExtendTorso, LookAtBottle, Wait, UpdateBottlePose], [None, None, 4, None])

class BottleGrasped(AbstractDecisionElement):
    """
    Graps the bottle
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)        
        blackboard.manipulation_iteration = 0

    def perform(self, blackboard, reevaluate=False):
        if blackboard.bottle_grasped:
            return self.push(InPouringPosition)
        if blackboard.manipulation_iteration >= 2:
            return self.push_action_sequence(SequenceElement, [LookAtBottle, Wait, UpdateBottlePose, PickUpBottle], [None, 4, None, None])
        else:
            return self.push(PickUpBottle)

class InPouringPosition(AbstractDecisionElement):
    """
    Goes to pouring position
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)        
        blackboard.last_redoable = blackboard.POUR

    def perform(self, blackboard, reevaluate=False):
        if blackboard.redo_requested and blackboard.last_redoable == blackboard.POUR:
            blackboard.redo_requested = False
            return self.push(MoveToPouringPosition)

        if blackboard.arrived_at_pouring_position:
            # fill in liquid and wait a moment to see if redo card was shown
            return self.push(GlassLocated)
        else:
            return self.push(MoveToPouringPosition)

    def get_reevaluate(self):
        return True

class GlassLocated(AbstractDecisionElement):
    """
    Locates the position of the glass to be able to pour
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)
        blackboard.glass_located = False
        blackboard.glass_not_found = False
        blackboard.manipulation_iteration = 0

    def perform(self, blackboard, reevaluate=False):
        if blackboard.manipulation_iteration >= 3:
            return self.push_action_sequence(SequenceElement, [ExtendTorso, LookAtGlass, Wait, UpdateGlassPose ,PourLiquid], [None, None, 4, None ,None])
        if blackboard.glass_located:
            return self.push_action_sequence(SequenceElement, [PourLiquid], [None])
        if blackboard.glass_not_found:
            return self.push(SayGlassNotFound)
        else:
            return self.push_action_sequence(SequenceElement, [ExtendTorso, LookAtGlass, Wait, UpdateGlassPose], [None, None, 4, None])
