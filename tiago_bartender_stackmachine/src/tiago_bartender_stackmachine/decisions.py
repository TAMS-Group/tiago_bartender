from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from .actions import IdleMoveAround, Noop

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
    def perform(self, blackboard, reevaluate=False):
        if blackboard.redo_requested and blackboard.last_redoable == blackboard.TAKE_ORDER:
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
    
    """
    def perform(self, blackboard, reevaluate=False):
        pass
