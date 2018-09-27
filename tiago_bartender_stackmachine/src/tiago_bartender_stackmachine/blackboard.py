import rospy
import actionlib
from person_detection.msg import PersonDetections
from tiago_bartender_msgs.msg import PlaceAction, PourAction, PickAction, MoveToTargetAction, TakeOrderAction, DetectBottlesAction, DetectGlassAction
from tiago_bartender_msgs.srv import LookAt
from control_msgs.msg import FollowJointTrajectoryAction
from pal_interaction_msgs.msg import TtsAction
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import Bool, String
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import ObjectColor
from tf import TransformListener
import math


class Blackboard:
    def __init__(self):
        # TODO: implement with command cards listener
        self.tf_listener = TransformListener()

        # initialize variables
        self.reset()
        self.was_pause_card_shown = False

        self.redo_requested = False

        self.manipulation_iteration = 0

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
        self.detect_glass_action_client = None

        # service clients
        self.look_at_service = None
        self.planning_scene_service = None

        # publishers
        self.person_detection_switch_pub = None

        # subscribers
        self.person_detections_sub = rospy.Subscriber('person_detection/person_detections', PersonDetections, self.person_detections_cb)
        self.command_cards_sub = rospy.Subscriber('/command_cards/commands', String, self.command_cards_cb)

        self.last_pause_detection = rospy.Time.now()

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
        self.recipes = rospy.get_param('tiago_bartender/recipes')
        self.idle_zone = rospy.get_param('tiago_bartender/idle_zone')
        self.home_pose = rospy.get_param('tiago_bartender/home_pose')
        self.take_order_pose = rospy.get_param('tiago_bartender/take_order_pose')
        self.place_bottle_offset = rospy.get_param('tiago_bartender/place_bottle_offset')

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
        self.place_action_client = actionlib.SimpleActionClient('tiago_place', PlaceAction)

        rospy.loginfo("Initilizing pouring action client")
        self.pour_action_client = actionlib.SimpleActionClient('tiago_pour', PourAction)

        rospy.loginfo("Initializing detect bottles action client")
        self.detect_bottles_action_client = actionlib.SimpleActionClient('detect_bottles_action', DetectBottlesAction)

        rospy.loginfo("Initializing detect bottles action client")
        self.detect_glass_action_client = actionlib.SimpleActionClient('detect_glass_action', DetectGlassAction)

        # init LookAtService
        rospy.loginfo("Initializing look_at service client")
        self.look_at_service = rospy.ServiceProxy("head_controller/look_at_service", LookAt)
        self.planning_scene_service = rospy.ServiceProxy("apply_planning_scene", ApplyPlanningScene)
        self.person_detection_switch_pub = rospy.Publisher("person_detection/set_enabled", Bool, queue_size=1, latch=True)

    def person_detections_cb(self, detections):
        #TODO: Check if customer in front of bar
        if len(detections.detections) > 0:
            self.person_detected = True
            self.person_position = detections.detections[0].position

    def command_cards_cb(self, command):
        if command.data == 'pause':
            if (rospy.Time.now() - self.last_pause_detection) > rospy.Duration.from_sec(10.0):
                self.last_pause_detection = rospy.Time.now()
                self.is_paused = not self.is_paused

    def reset_for_next_bottle(self):
        self.arrived_at_pouring_position = False
        self.arrived_at_bottle = False
        self.bottle_not_found = False
        self.bottle_located = False
        self.glass_not_found = False
        self.glass_located = False
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
        self.bottle_located = False
        self.glass_not_found = False
        self.glass_located = False
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

    def add_invisible_collision_object(self):
        co_box = CollisionObject()
        co_box.header.frame_id = 'base_footprint'
        co_box.id = 'invisible_box'
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box_height = 0.76
        box.dimensions.append(0.80)
        box.dimensions.append(1.60)
        box.dimensions.append(box_height)
        co_box.primitives.append(box)
        box_pose = Pose()
        box_pose.position.x = 0.80
        box_pose.position.y = 0.0
        box_pose.position.z = box_height/2.0
        box_pose.orientation.w = 1.0
        co_box.primitive_poses.append(box_pose)
        co_box.operation = CollisionObject.ADD
        color = ObjectColor()
        color.id = 'invisible_box'
        color.color.g = 1.0
        color.color.a = 0.15

        ps = PlanningScene()
        ps.world.collision_objects.append(co_box)
        ps.object_colors.append(color)
        ps.is_diff = True
        try:
            self.planning_scene_service(ps)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    def remove_invisible_collision_object(self):
        co_box = CollisionObject()
        co_box.id = 'invisible_box'
        co_box.operation = CollisionObject.REMOVE

        ps = PlanningScene()
        ps.world.collision_objects.append(co_box)
        ps.is_diff = True
        try:
            self.planning_scene_service(ps)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)

    def rotate_point(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.
        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = point

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy
