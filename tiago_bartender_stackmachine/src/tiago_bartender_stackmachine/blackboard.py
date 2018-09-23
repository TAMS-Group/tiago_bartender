import rospy
import actionlib
from person_detection.msg import PersonDetections
from tiago_bartender_msgs.msg import PlaceAction, PourAction, PickAction, MoveToTargetAction, TakeOrderAction, DetectBottlesAction
from tiago_bartender_msgs.srv import LookAt
from control_msgs.msg import FollowJointTrajectoryAction
from pal_interaction_msgs.msg import TtsAction
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import Bool

class Blackboard:
    def __init__(self):
        # TODO: implement with command cards listener
        self.tf_listener = TransformListener()

        # initialize variables
        self.reset()
        self.was_pause_card_shown = False

        self.redo_requested = False

        self.last_redoable = None
        self.TAKE_ORDER = 1
        self.PICK = 2
        self.POUR = 3

        # action clients
        self.move_action_client = None
        self.take_order_action_client = None
        self.grasp_action_client = None
        self.pour_action_client = None
        self.torso_action_client = None
        self.tts_action_client = None
        self.move_base_action_client = None
        self.detect_bottles_action_client = None

        # service clients
        self.look_at_service = None

        # publishers
        self.person_detection_switch_pub = None

        # subscribers
        self.person_detections_sub = rospy.Subscriber('person_detection/person_detections', PersonDetections, self.person_detections_cb)

        # parameters
        self.recipes = None
        self.idle_zone = None
        self.home_pose = None
        self.take_order_pose = None
        self.place_bottle_offset = None

        # saved poses
        self.person_position = None
        self.last_bottle_pose = None

        rospy.loginfo("Load recipes")
        self.recipes = rospy.get_param('~recipes')
        self.idle_zone = rospy.get_param('~idle_zone')
        self.home_pose = rospy.get_param('~home_pose')
        self.take_order_pose = rospy.get_param('~take_order_pose')
        self.place_bottle_offset = rospy.get_param('~place_bottle_offset')

        # we initlizie all the action clients
        rospy.loginfo("Initilizing move to target action client")
        self.move_action_client = actionlib.SimpleActionClient('move_to_target', MoveToTargetAction)

        rospy.loginfo("Initilizing follow joint trajectory action client")
        self.torso_action_client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        rospy.loginfo("Initilizing tts action client")
        self.tts_action_client = actionlib.SimpleActionClient('tts', TtsAction)

        rospy.loginfo("Initilizing take order action client")
        self.take_order_action_client = actionlib.SimpleActionClient('menu/take_order', TakeOrderAction)

        rospy.loginfo("Initilizing move_base action client")
        self.move_base_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Initilizing pick action client")
        self.pick_action_client = actionlib.SimpleActionClient('tiago_pick', PickAction)

        rospy.loginfo("Initilizing place action client")
        self.pick_action_client = actionlib.SimpleActionClient('tiago_place', PlaceAction)

        rospy.loginfo("Initilizing pouring action client")
        self.pour_action_client = actionlib.SimpleActionClient('tiago_pour', PourAction)

        rospy.loginfo("Initializing detect bottles action client")
        self.detect_bottles_action_client = actionlib.SimpleActionClient('detect_bottles_action', DetectBottlesAction)

        # init LookAtService
        rospy.loginfo("Initializing look_at service client")
        self.look_at_service = rospy.ServiceProxy("head_controller/look_at_service", LookAt)
        self.person_detection_switch_pub = rospy.Publisher("person_detection/set_enabled", Bool, queue_size=10)

    def person_detections_cb(self, detections):
        #TODO: Check if customer in front of bar
        if detections:
            self.person_detected = True
        for p in detections:
            self.person_position = p.position

    def reset_for_next_bottle(self):
        self.arrived_at_pouring_position = False
        self.arrived_at_bottle = False
        self.bottle_not_found = False
        self.get_next_bottle = True
        self.got_next_bottle = False
        self.no_menu_found = False

    def reset(self):
        self.has_customer = False
        self.person_detected = False

        self.arrived_at_customer = False
        self.arrived_at_bottle = False
        self.arrived_at_pouring_position = False
        self.bottle_not_found = False
        self.is_paused = False
        self.bottle_grasped = False
        self.arrived_at_pouring_position = False
        self.get_next_bottle = False
        self.got_next_bottle = False

        # We tried to find a menu, but failed
        self.no_menu_found = False

        # list of tuples with liquid name and quantity
        self.recipe = None
        self.current_drink = None
        self.current_bottle = None
        self.current_pour_time = None
