import rospy
import actionlib
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from bitbots_stackmachine.sequence_element import SequenceElement
from .actions import IdleMoveAround, WaitingToResume, MoveToCustomer, SayRepeatOrder, SayNoMenuFoundRepeat, SayOrderConfirmed, ObserveOrder, LookAtCustomer, SayPleaseOrder, LookAtMenu, MoveToBottle, LookAtBottle, MoveToPouringPosition, PourLiquid, Wait, PickUpBottle, SayDrinkFinished
import tiago_bartender_msgs.action 


# @BitBots: see IMPROVE tags


class Init(AbstractDecisionElement):
    """
    Initializes the tiago bartender demo.
    Verifies ros connectivity
    """
    def __init__(self, blackboard):
        # we initlizie all the action clients
        rospy.loginfo("Initilizing move action client")
        blackboard.move_action_client = actionlib.SimpleActionClient('move', MoveToTarget)
        blackboard.move_action_client.wait_for_server()

        rospy.loginfo("Initilizing take order action client")
        blackboard.take_order_action_client = actionlib.SimpleActionClient('take_order', TakeOrder)
        blackboard.take_order_action_client.wait_for_server()

        rospy.loginfo("Initilizing pick action client")
        blackboard.pick_action_client = actionlib.SimpleActionClient('pick', Pick)
        blackboard.pick_action_client.wait_for_server()

        rospy.loginfo("Initilizing pouring action client")
        blackboard.pour_action_client = actionlib.SimpleActionClient('pour', Pour)
        blackboard.pour_action_client.wait_for_server()

        # init LookAtService
        blackboard.look_at_service = None #TODO init

    def perform(self, blackboard, reevaluate=False):
        return self.push(Paused)


# TODO: refactor as AbstractPausableAction
class Paused(AbstractDecisionElement):
    """
    Check whether the system should is paused.
    """
    def __init__(self):
        self.saved_stack = None

    def get_reevaluate(self):
        return True

    def perform(self, blackboard):
        if blackboard.was_pause_card_shown:
            blackboard.was_pause_card_shown = False
            if self.saved_stack:
                # see if we already saved a stack (meaning we're currently paused)
                self._behaviour = self.saved_stack
                self.saved_stack = None
                return    
            else:
                # IMPROVE: the stack should *not* be called _behaviour, especially not in BE            
                self.saved_stack = self._behaviour
                return self.push(WaitingToResume)        
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


class Idle(AbstractDecisionElement):
    """
    Act in idle mode, checking for customer every so often
    """
    def perform(self, blackboard, reevaluate=False):
        # TODO: implement alternatives
        return self.push(IdleMoveAround)



class InFrontOfCustomer(AbstractDecisionElement):
    """
    Decide whether to take order or move to customer first
    """
    def __init__(self, blackboard, _):
        blackboard.last_redoable = blackboard.TAKE_ORDER

    def perform(self, blackboard, reevaluate=False):
        if blackboard.redo_requested and blackboard.last_redoable == blackboard.TAKE_ORDER and self.arrived_at_customer:

           blackboard.redo_requested = False
           blackboard.arrived_at_customer = False
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
        self.order_confirmed = False

    def perform(self, blackboard, reevaluate=False):
        if blackboard.order:
            if self.order_confirmed:
                return self.push(MakeCocktail)
            else:
                self.order_confirmed = True
                return self.push(SayOrderConfirmed)
        elif blackboard.no_menu_found:
            return self.push_action_sequence(SequenceElement, [SayNoMenuFoundRepeat, SayOrderConfirmed, ObserveOrder, LookAtCustomer], [None, None, None, None])
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
        self.first = True

    def perform(self, blackboard, reevaluate=False):
        if self.first:
            self.first = False
            return self.push(SayDrinkFinished)
        else:
            return self.push(CleanUp)

class CleanUp(AbstractDecisionElement):
    """
    Put away last bottle and go back to init via interrupt.
    """

    def perform(self, blackboard, reevaluate=False):
        if blackboard.has_bottle_in_hand:
            # TODO put bottle away
            pass
        else:
            # we call an interrupt to get completely back to init
            return self.interrupt()


class InFrontOfRequiredBottle(AbstractDecisionElement):
    """
    Decides if the robot is in front of the bottle
    """
    def __init__(self, blackboard, _):
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
