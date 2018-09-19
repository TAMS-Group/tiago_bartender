from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from .actions import IdleMoveAround, Noop, MoveToCustomer, SayRepeatOrder

# @BitBots: see IMPROVE tags


class Init(AbstractDecisionElement):
    """
    Initializes the tiago bartender demo.
    Verifies ros connectivity
    """
    def perform(self, blackboard, reevaluate=False):
        return self.push(Paused)

    def get_reevaluate(self):
        return True



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
        if blackboard.is_paused:
            # IMPROVE: the stack should *not* be called _behaviour, especially not in BE
            self.saved_stack = self._behaviour
            return self.push(Noop)
        elif self.saved_stack:
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
            return self.push(InFrontOfCustomer)
        else:
            return self.push(Idle)



class Idle(AbstractDecisionElement):
    """
    Act in idle mode, checking for customer every so often
    """
    # IMPROVE: reevaluate should not
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
        if blackboard.redo_requested and
           blackboard.last_redoable == blackboard.TAKE_ORDER and
           self.arrived_at_customer:

            blackboard.redo_requested = False
            blackboard.arrived_at_customer = False
            return self.push_action_sequence([SayRepeatOrder, MoveToCustomer])

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
            return self.push_action_sequence([SayNoMenuFoundRepeat, LookAtMenu, ObserveOrder, LookAtCustomer])
        else:
            return self.push_action_sequence([SayPleaseOrder, LookAtMenu, ObserveOrder, LookAtCustomer])



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


class InFrontOfRequiredBottle(AbstractDecisionElement):
    """
    Decides if the robot is in front of the bottle
    """
    def __init__(self, blackboard, _):
        blackboard.last_redoable = blackboard.PICK

    def perform(self, blackboard, reevaluate=False):
        if blackboard.redo_requested and blackboard.last_redoable == blackboard.PICK:
            #TODO maybe go back to init position
            return self.push_action_sequence([SayRepeatOrder, MoveToBottle])
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
            return self.push(GrapsBottle)


class InPouringPosition(AbstractDecisionElement):
    """
    Goes to pouring position
    """

    def perform(self, blackboard, reevaluate=False):
        if blackboard.in_pouring_position:
            return self.push(FillLiquide)
        else:
            return self.push(MoveToPouringPosition)
 
