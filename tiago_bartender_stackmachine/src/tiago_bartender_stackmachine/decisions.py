import rospy
import actionlib
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from bitbots_stackmachine.sequence_element import SequenceElement
from .actions import IdleMoveAround, WaitingToResume, MoveToCustomer, SayRepeatOrder, SayNoMenuFoundRepeat, SayOrderConfirmed, ObserveOrder, LookAtCustomer, SayPleaseOrder, LookAtMenu, MoveToBottle, LookAtBottle, MoveToPouringPosition, PourLiquid, Wait, PickUpBottle, SayDrinkFinished
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
        self.initilized = False

    def perform(self, blackboard, reevaluate=False):
        if not blackboard.move_action_client.wait_for_server(0.01):
            return self.push(WaitForRos, 'move_to_target')
        if not blackboard.torso_action_client.wait_for_server(0.01):
            return self.push(WaitForRos, 'torso_controller/follow_joint_trajectory')
        if not blackboard.tts_action_client.wait_for_server(0.01):
            return self.push(WaitForRos, 'tts')
        if not blackboard.take_order_action_client.wait_for_server(0.01):
            return self.push(WaitForRos, 'menu/take_order')
        if not blackboard.move_base_action_client.wait_for_server(0.01):
            return self.push(WaitForRos, 'move_base')
        if not blackboard.pick_action_client.wait_for_server(0.01):
            return self.push(WaitForRos, 'pick')
        if not blackboard.pour_action_client.wait_for_server(0.01):
            return self.push(WaitForRos, 'pour')
        if not rospy.wait_for_service('head_controller/look_at_service', 0.01):
            return self.push(WaitForRos, 'head_controller/look_at_service')
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
                return self.push(HasCustomer)


class HasCustomer(AbstractDecisionElement):
    """
    Decide what to do based on whether we currently have a customer
    """
    def perform(self, blackboard, reevaluate=False):
        if blackboard.has_customer:
            #TODO maybe the blackboard.has_customer has to be resettet to False
            return self.push(InFrontOfCustomer)
        else:
            return self.push(Idle)

    def get_reevaluate(self):
        return True


class Idle(AbstractDecisionElement):
    """
    Act in idle mode, checking for customer every so often
    """
    def perform(self, blackboard, reevaluate=False):
        # TODO: implement alternatives
        return self.push_action_sequence(SequenceElement, [LookForward, IdleMoveAround, LookForCustomer], [None, None])



class InFrontOfCustomer(AbstractDecisionElement):
    """
    Decide whether to take order or move to customer first
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)
        blackboard.last_redoable = blackboard.TAKE_ORDER
        blackboard.arrived_at_customer = False

    def perform(self, blackboard, reevaluate=False):
        if blackboard.redo_requested and blackboard.last_redoable == blackboard.TAKE_ORDER and blackboard.arrived_at_customer:

           blackboard.redo_requested = False
           blackboard.arrived_at_customer = False
           #TODO: maybe remove MoveToCustomer here
           return self.push_action_sequence(SequenceElement, [SayRepeatOrder, MoveToCustomer], [None, None])

        if blackboard.arrived_at_customer:
            return self.push(TakeOrder)
        else:
            return self.push(MoveToCustomer)

    # IMPROVE: you want to allow to have logic in this decision, but
    # 1) is this really necessary or can you use as static variable instead?
    # 2) the logic probably would depend on the blackboard, which is not given here
    def get_reevaluate(self):
        return True



class TakeOrder(AbstractDecisionElement):
    """
    Let the robot take an order from the human
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)        
        self.order_confirmed = False
        blackboard.recipe = None
        blackboard.no_menu_found = False

    def perform(self, blackboard, reevaluate=False):
        if blackboard.recipe:
            if self.order_confirmed:
                return self.push(MakeCocktail)
            else:
                self.order_confirmed = True
                return self.push(SayOrderConfirmed)
        elif blackboard.no_menu_found:
            blackboard.no_menu_found = False
            return self.push_action_sequence(SequenceElement, [SayNoMenuFoundRepeat, LookAtMenu, ObserveOrder, LookAtCustomer], [None, None, None, None])
        else:
            return self.push_action_sequence(SequenceElement, [SayPleaseOrder, LookAtMenu, ObserveOrder, LookAtCustomer], [None, None, None, None])


class MakeCocktail(AbstractDecisionElement):
    """
    Make a cocktail out of multiple ingrediances
    """
    def perform(self, blackboard, reevaluate=False):
        if len(blackboard.recipe)>0:
            # add next liquid
            return self.push(InFrontOfRequiredBottle)
        else:
            # we're finished
            return self.push(DrinkFinished) #TODO: say something & place bottle back

class DrinkFinished(AbstractDecisionElement):
    """
    The Drink is finished. Tell it to the costumer and clean up
    """
    def __init__(self, blackboard):
        super(DrinkFinished, self).__init__(blackboard)
        self.first = True

    def perform(self, blackboard, reevaluate=False):
        if self.first:
            self.first = False
            return self.push(SayDrinkFinished)
        else:
            return self.push(PutBottleBack)

class PutBottleBack(AbstractDecisionElement):
    """
    Put away last bottle and go back to init via interrupt.
    """
    def perform(self, blackboard, reevaluate=False):
        if not blackboard.arrived_at_bottle:
            # TODO put bottle away
            first_iteration = False
            return self.push_action_sequence(SequenceElement, [LookForward, MoveToBottlePose], [None, None])
        elif not blackboard.placed_last_bottle:
            return self.push(PlaceBottle)
        else:
            # resetting variables in blackboard and going back to HasCustomer
            blackboard.reset()


class InFrontOfRequiredBottle(AbstractDecisionElement):
    """
    Decides if the robot is in front of the bottle
    """
    def __init__(self, blackboard, _):
        super(AbstractDecisionElement, self).__init__(blackboard)        
        blackboard.last_redoable = blackboard.PICK

    def perform(self, blackboard, reevaluate=False):
        if blackboard.redo_requested and blackboard.last_redoable == blackboard.PICK:
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

    def perform(self, blackboard, reevaluate=False):
        if blackboard.bottle_located:
            return self.push(BottleGrasped)
        else:
            return self.push(LookAtBottle)

class BottleGrasped(AbstractDecisionElement):
    """
    Graps the Bottle
    """

    def perform(self, blackboard, reevaluate=False):
        if blackboard.bottle_grapsed:
            return self.push(InPouringPosition)
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
            return self.push(MoveToPouringPosition)

        if blackboard.in_pouring_position:
            # fill in liquid and wait a moment to see if redo card was shown
            return self.push_action_sequence(SequenceElement, [PourLiquid, Wait], [None, 5])
        else:
            return self.push(MoveToPouringPosition)

    def get_reevaluate(self):
        return True
