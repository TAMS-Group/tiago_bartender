
class Blackboard:
    def __init__(self):
        # TODO: implement with command cards listener
        self.is_paused = False

        self.arrived_at_customer = False

        self.last_redoable = None
        self.TAKE_ORDER = 1
        self.PICK = 2
        self.POUR = 3

        # list of tuples with liquide name and quantity
        self.order = None