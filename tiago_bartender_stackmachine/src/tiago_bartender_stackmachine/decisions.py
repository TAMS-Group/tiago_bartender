from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from .actions import IdleMoveAround

class Init(AbstractDecisionElement):
    """
    Initializes the tiago bartender demo.
    Verifies ros connectivity
    """
    def perform(self, blackboard, reevaluate=False):
        return self.push(Idle)

    def get_reevaluate(self):
        return True

class Idle(AbstractDecisionElement):
    """
    Implements idle behavior
    """
    def perform(self, blackboard, reevaluate=False):
        return self.push(IdleMoveAround)

