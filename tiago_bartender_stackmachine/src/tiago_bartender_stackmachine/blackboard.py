
class Blackboard:
    def __init__(self):
        # TODO: implement with command cards listener
        self.is_paused = False

        self.arrived_at_customer = False

        self.last_redoable = None
        self.TAKE_ORDER = 1
                self.PICK = 2
        self.POUR = 3

        # We tried to find a menu, but failed
        self.no_menu_found = False

        # list of tuples with liquid name and quantity
        self.recipe = None

    def order(self, drink):
        self.recipe = [("rum", 5), ("tequila", 10)]
