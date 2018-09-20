
class Blackboard:
    def __init__(self):
        # TODO: implement with command cards listener
        self.was_pause_card_shown = False

        self.arrived_at_customer = False
        self.arrived_at_bottle = False
        self.arrived_at_pouring_position = False

        self.last_redoable = None
        self.TAKE_ORDER = 1
        self.PICK = 2
        self.POUR = 3

        # We tried to find a menu, but failed
        self.no_menu_found = False

        # list of tuples with liquid name and quantity
        self.recipe = None

        # action clients
        self.move_action_client = None
        self.take_order_action_client = None
        self.grasp_action_client = None
        self.pour_action_client = None

        self.look_at_service = None

    def order(self, drink):
        self.recipe = [("rum", 5), ("tequila", 10)]
