from __future__ import print_function

import copy
import rospy
import actionlib
import random
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
from tiago_bartender_msgs.msg import PlaceGoal, PourGoal, PickGoal, TakeOrderGoal, MoveToTargetGoal, DetectBottlesGoal, ManipulationResult, DetectGlassGoal
from random import uniform
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Bool
from pal_interaction_msgs.msg import TtsGoal
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryGoal, JointTolerance, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf

class IdleMoveAround(AbstractActionElement):
    """
    move to random pose behind the counter
    """
    def __init__(self, blackboard, _):
        super(IdleMoveAround, self).__init__(blackboard)
        self.first_iteration = True
        self.repeat = False

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = blackboard.idle_zone['frame']
        self.goal_x = uniform(blackboard.idle_zone['center_x'] - blackboard.idle_zone['radius_x'], blackboard.idle_zone['center_x'] + blackboard.idle_zone['radius_x'])
        self.goal_y = uniform(blackboard.idle_zone['center_y'] - blackboard.idle_zone['radius_y'], blackboard.idle_zone['center_y'] + blackboard.idle_zone['radius_y'])

        origin = (blackboard.idle_zone['center_x'], blackboard.idle_zone['center_y'])
        point = (self.goal_x, self.goal_y)
        angle = blackboard.idle_zone['euler_z']
        self.goal_x, self.goal_y = blackboard.rotate_point(origin, point, angle)
        self.goal.target_pose.pose.position.x = self.goal_x
        self.goal.target_pose.pose.position.y = self.goal_y
        self.goal.target_pose.pose.orientation.w = 1.0

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration or self.repeat:
            print("IdleMoveAround")
            blackboard.move_base_action_client.send_goal(self.goal)
            self.repeat = False
            self.first_iteration = False

        # wait till action is completed
        if not blackboard.move_base_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return
        state = blackboard.move_base_action_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            self.pop()
        else:
            self.repeat = True


class WaitForRos(AbstractActionElement):
    def __init__(self, blackboard, args=''):
        super(WaitForRos, self).__init__(blackboard)
        self.first_iteration = True
        self.name = args
    def perform(self, blackboard, reevaluate=False):
        rospy.loginfo_throttle(10, "Waiting for server %s"% self.name)
        self.pop()


class WaitingToResume(AbstractActionElement):
    """
    This action doesn't do anything and is only put on the stack for visualization purposes
    """
    def perform(self, blackboard, reevaluate=False):
        rospy.loginfo_throttle(10, "I'm currently paused. Please show pause card to unpause me.")


class MoveToCustomer(AbstractActionElement):
    def __init__(self, blackboard, _):
        super(MoveToCustomer, self).__init__(blackboard)
        self.first_iteration = True
        self.repeat = False
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = blackboard.take_order_pose['frame']
        self.goal.target_pose.pose.position.x = blackboard.take_order_pose['pos_x']
        self.goal.target_pose.pose.position.y = blackboard.take_order_pose['pos_y']
        self.goal.target_pose.pose.orientation.x = blackboard.take_order_pose['ori_x']
        self.goal.target_pose.pose.orientation.y = blackboard.take_order_pose['ori_y']
        self.goal.target_pose.pose.orientation.z = blackboard.take_order_pose['ori_z']
        self.goal.target_pose.pose.orientation.w = blackboard.take_order_pose['ori_w']

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration or self.repeat:
            print("MoveToCustomer")
            blackboard.move_base_action_client.send_goal(self.goal)
            self.first_iteration = False
            self.repeat = False

        # wait till action is completed
        if not blackboard.move_base_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return
        state = blackboard.move_base_action_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            blackboard.arrived_at_customer = True
        else:
            self.repeat = True

class MoveToBottle(AbstractActionElement):
    """
    Moves to the pose to put down the last bottle of the recipe
    """
    def __init__(self, blackboard, _):
        super(MoveToBottle, self).__init__(blackboard)
        self.first_iteration = True
        self.repeat = False
        blackboard.arrived_at_bottle = False
        self.goal = MoveToTargetGoal()
        self.goal.target = blackboard.current_bottle
        self.goal.look_at_target = False

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration or self.repeat:
            print('MoveToBottle')
            blackboard.move_action_client.send_goal(self.goal)
            self.first_iteration = False
            self.repeat = False

        # wait till action is completed
        if not blackboard.move_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return
        state = blackboard.move_action_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            result = blackboard.move_action_client.get_result()
            blackboard.last_bottle_pose = result.target_pose_result;
            blackboard.arrived_at_bottle = True
            self.pop()
        else:
            self.repeat = True

class MoveToPouringPosition(AbstractActionElement):
    """
    Moves to the pose to put down the last bottle of the recipe
    """
    def __init__(self, blackboard, _):
        super(MoveToPouringPosition, self).__init__(blackboard)
        self.first_iteration = True
        self.repeat = False
        blackboard.arrived_at_pouring_position = False
        self.goal = MoveToTargetGoal()
        self.goal.target = 'glass'
        self.goal.look_at_target = False

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration or self.repeat:
            print('MoveToPouringPosition')
            blackboard.move_action_client.send_goal(self.goal)
            self.first_iteration = False
            self.repeat = False

        # wait till action is completed
        if not blackboard.move_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return
        state = blackboard.move_action_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            blackboard.arrived_at_pouring_position = True
            self.pop()
        else:
            self.repeat = True

class AbstractLookAt(AbstractActionElement):
    def __init__(self, blackboard, _):
        super(AbstractLookAt, self).__init__(blackboard)
        self.blackboard = blackboard

    def perform(self, blackboard, reevaluate=False):
        target = self.target()
        pose = PointStamped()
        print("Looking at " + target)
        try:
            blackboard.look_at_service(target, pose)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
        self.pop()

    def target(self):
        raise NotImplementedError


class LookAtCustomer(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        target = "customer"
	point = PointStamped()
	point.header.frame_id = 'xtion_optical_frame'
	point.point = blackboard.customer_position
        print("Looking at " + target)
        try:
            blackboard.look_at_service(target, point)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
        self.pop()

class LookAtBottle(AbstractLookAt):
    def target(self):
        return self.blackboard.current_bottle

class SearchBottleLeft(AbstractLookAt):
    def target(self):
        return 'search_bottle_left'

class SearchBottleRight(AbstractLookAt):
    def target(self):
        return 'search_bottle_right'

class LookAtGlass(AbstractLookAt):
    def target(self):
        return "glass"

class LookAtMenu(AbstractLookAt):
    def target(self):
        return "down"

class LookForward(AbstractLookAt):
    def target(self):
        return "forward"

class LookDefault(AbstractLookAt):
    def target(self):
        return "default"

class LookAtPlacePose(AbstractActionElement):
    def perform(self, blackboard, reevaluate=False):
        target = ""
	point = PointStamped()
	point.header.frame_id = 'xtion_optical_frame'
	point.point = blackboard.last_bottle_pose
        print("Looking at place pose")
        try:
            blackboard.look_at_service(target, point)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
        self.pop()

class LookForCustomer(AbstractActionElement):
    def __init__(self, blackboard, _):
        super(LookForCustomer, self).__init__(blackboard)
        self.first_iteration = True

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration:
            print('LookForCustomer')
            target = 'look_around'
            pose = PointStamped()
            try:
                blackboard.look_at_service(target, pose)
            except rospy.ServiceException, e:
                print("Service call failed: %s"%e)

            self.begin = rospy.get_rostime()

            enable = Bool()
            enable.data = True
            blackboard.person_detection_switch_pub.publish(enable)
            self.first_iteration = False

        if blackboard.person_detected:
            disable = Bool()
            disable.data = False
            blackboard.person_detection_switch_pub.publish(disable)

            blackboard.customer_position = blackboard.person_position
            blackboard.has_customer = True
            blackboard.person_detected = False
        elif rospy.get_rostime() - self.begin >= rospy.Duration.from_sec(60.0):
            disable = Bool()
            disable.data = False
            blackboard.person_detection_switch_pub.publish(disable)

            self.pop()

class AbstractSay(AbstractActionElement):
    def __init__(self, blackboard, _):
        super(AbstractSay, self).__init__(blackboard)
        self.blackboard = blackboard
        self.first_iteration = True
        text = random.choice(self.text())
        self.goal = TtsGoal()
        self.goal.rawtext.text = text
        self.goal.rawtext.lang_id = "en_GB"

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration:
            print("Saying '" + self.goal.rawtext.text + "'")
            blackboard.tts_action_client.send_goal(self.goal)
            self.first_iteration = False
        state = blackboard.tts_action_client.get_state()
        # wait till action is completed
        if state == GoalStatus.SUCCEEDED:
            self.pop()

    def text(self):
        raise NotImplementedError

class SayPleaseOrder(AbstractSay):
    def text(self):
        return ["Welcome. You can order by pointing at the drink of your choice on the menu card.",
                "Hello there. Please order by pointing at one of the drinks on the menu card."]

class SayAcid(AbstractSay):
    def text(self):
        self.blackboard.current_drink = ''
        return ["I am sorry. We only serve this to robots. But if you'd like you can choose a different drink.",
                "I am sorry. But I don't think your weak human body will be able to handle this. Please choose a different drink."]

class SayNoMenuFoundRepeat(AbstractSay):
    def text(self):
        return ["I thought I put the menu card here? Could you help me and put the menu card in front of me.", 
                "I was sure the menu card was right here. Could you help me by putting it in front of me.", 
                "Where did I put the menu again? Please help me and put it in front of me, so I can see it."]

class SayRepeatOrder(AbstractSay):
    def text(self):
        return ["Oh, so you want something different?"]

class SayOrderConfirmed(AbstractSay):
    def text(self):
        return ["One " + self.blackboard.current_drink + " coming right up."]

class SayDrinkFinished(AbstractSay):
    def text(self):
        return ["Here you go!",
                "Enjoy your drink!",
                "Here's your drink!"]

class SayBottleNotFound(AbstractSay):
    def text(self):
        self.blackboard.bottle_not_found = False
        return ["I was sure I put the " + self.blackboard.current_bottle + " right here. Please help me by putting the bottle in front of me."]


class SayGlassNotFound(AbstractSay):
    def text(self):
        self.blackboard.glass_not_found = False
        return ["I was sure I put the glass right here. Please help me by putting the it in front of me."]

class ObserveOrder(AbstractActionElement):
    def __init__(self, blackboard, _):
        super(ObserveOrder, self).__init__(blackboard)
        self.first_iteration = True
        self.goal = TakeOrderGoal()
        self.goal.timeout = rospy.Duration.from_sec(20)

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration:
            print('ObserveOrder')
            blackboard.take_order_action_client.send_goal(self.goal)
            self.first_iteration = False

        # if no result yet
        if not blackboard.take_order_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return
        blackboard.no_menu_found = False
        result = blackboard.take_order_action_client.get_result()
        if result.status == "timeout":
            self.pop()
        elif result.status == "no_menu_card_detected":
            blackboard.no_menu_found = True
            self.pop()
        else:
            blackboard.current_drink = result.selection
            if blackboard.current_drink == 'sulfuric_acid':
                self.pop()
            else:
                blackboard.recipe = copy.deepcopy(blackboard.recipes[result.selection])
                self.pop()

class UpdateBottlePose(AbstractActionElement):
    def __init__(self, blackboard, _):
        super(UpdateBottlePose, self).__init__(blackboard)
        self.first_iteration = True
        self.goal = DetectBottlesGoal()
        self.goal.timeout = rospy.Duration.from_sec(10)
        self.goal.stability_threshold = 5

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration:
            print('UpdateBottlePose')
            blackboard.detect_bottles_action_client.send_goal(self.goal)
            self.first_iteration = False
        # if no result yet
        if not blackboard.detect_bottles_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return

        blackboard.bottle_located = False
        result = blackboard.detect_bottles_action_client.get_result()
        if blackboard.current_bottle in result.detected_bottles:
            blackboard.bottle_located = True
            self.pop()
        else:
            blackboard.bottle_not_found = True
            self.pop()

class UpdateGlassPose(AbstractActionElement):
    def __init__(self, blackboard, _):
        super(UpdateGlassPose, self).__init__(blackboard)
        self.first_iteration = True
        self.goal = DetectGlassGoal()
        self.goal.timeout = rospy.Duration.from_sec(10)

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration:
            print('UpdateGlassPose')
            blackboard.detect_glass_action_client.send_goal(self.goal)
            self.first_iteration = False
        # if no result yet
        if not blackboard.detect_glass_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return

        blackboard.glass_located = False
        result = blackboard.detect_glass_action_client.get_result()
        if result.detected_glass == 'glass':
            print('glass located')
            blackboard.glass_located = True
            self.pop()
        else:
            print('glass not found')
            blackboard.glass_not_found = True
            self.pop()

class ExtendTorso(AbstractActionElement):
    """
    Extend the torso to maximum height
    """
    def __init__(self, blackboard, _):
        super(ExtendTorso, self).__init__(blackboard)
        self.first_iteration = True
        self.goal = FollowJointTrajectoryGoal()
        torso_command = JointTrajectory()
        torso_command.joint_names.append("torso_lift_joint")
        jtp = JointTrajectoryPoint()
        jtp.positions.append(0.35)
        jtp.time_from_start = rospy.Duration.from_sec(2.0)
        torso_command.points.append(jtp)
        self.goal.trajectory = torso_command
	jt = JointTolerance()
	jt.name = 'torso_lift_joint'
        jt.position = 0.01
        self.goal.goal_tolerance.append(jt)
        self.goal.goal_tolerance

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration:
            print('ExtendTorso')
            blackboard.torso_action_client.send_goal(self.goal)
            self.first_iteration = False

        if not blackboard.torso_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return
        result = blackboard.torso_action_client.get_result().error_code
        if result == FollowJointTrajectoryResult.SUCCESSFUL:
            self.pop()

class PickUpBottle(AbstractActionElement):
    """
    Calls the pick action
    """
    def __init__(self, blackboard, _):
        super(PickUpBottle, self).__init__(blackboard)
        self.first_iteration = True
        self.goal = PickGoal()
        self.goal.object_id = blackboard.current_bottle

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration:
            print("Trying to pick " + self.goal.object_id)
            blackboard.add_invisible_collision_object()
            blackboard.pick_action_client.send_goal(self.goal)
            self.first_iteration = False

        # if no result yet
        if not blackboard.pick_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return

        blackboard.manipulation_iteration = blackboard.manipulation_iteration + 1
        blackboard.remove_invisible_collision_object()
        result = blackboard.pick_action_client.get_result().result.result
        if result == ManipulationResult.SUCCESS:
            blackboard.bottle_grasped = True
            self.pop()
        elif result == ManipulationResult.UNREACHABLE:
            print("Bottle " + self.goal.object_id + " appears to be out of reach")
            self.pop()
        elif result == ManipulationResult.NO_PLAN_FOUND:
            print("Failed to plan pick for " + self.goal.object_id)
            self.pop()
        elif result == ManipulationResult.EXECUTION_FAILED:
            print("Execution of pick failed for " + self.goal.object_id)
            self.pop()

class PourLiquid(AbstractActionElement):
    """
    Calls the pouring action
    """
    def __init__(self, blackboard, _):
        super(PourLiquid, self).__init__(blackboard)
        self.first_iteration = True
        self.goal = PourGoal()
        self.goal.container_id = 'glass'

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration:
            print('PourLiquid')
            blackboard.add_invisible_collision_object()
            blackboard.pour_action_client.send_goal(self.goal)
            self.first_iteration = False

        # if no result yet
        if not blackboard.pour_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return

        blackboard.manipulation_iteration = blackboard.manipulation_iteration + 1
        blackboard.remove_invisible_collision_object()
        result = blackboard.pour_action_client.get_result().result.result
        if result == ManipulationResult.SUCCESS:
            print('pour liquid success')
            blackboard.reset_for_next_bottle()
            self.pop()
        elif result == ManipulationResult.UNREACHABLE:
            print('unreachable')
            self.pop()
        elif result == ManipulationResult.NO_PLAN_FOUND:
            print('no plan found')
            self.pop()
        elif result == ManipulationResult.EXECUTION_FAILED:
            print('execution failed')
            self.pop()

class PlaceBottle(AbstractActionElement):
    """
    Calls the place action
    """
    def __init__(self, blackboard, _):
        super(PlaceBottle, self).__init__(blackboard)
        self.first_iteration = True
        self.repeat = False
        self.goal = PlaceGoal()
        self.goal.place_pose = blackboard.last_bottle_pose

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration or self.repeat:
            print('PlaceBottle')
            blackboard.add_invisible_collision_object()
            blackboard.place_action_client.send_goal(self.goal)
            self.first_iteration = False
            self.repeat  = False

        # if no result yet
        if not blackboard.place_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return

        blackboard.remove_invisible_collision_object()
        result = blackboard.place_action_client.get_result().result.result
        if result == ManipulationResult.SUCCESS:
            blackboard.bottle_grasped = False
            self.pop()
        elif result == ManipulationResult.UNREACHABLE:
            print("Bottle place pose unreachable")
            self.repeat = True
        elif result == ManipulationResult.NO_PLAN_FOUND:
            print("No plan found for place")
            self.repeat = True
        elif result == ManipulationResult.EXECUTION_FAILED:
            print("Execution failed for place")
            self.repeat = True

class MoveToBottlePose(AbstractActionElement):
    """
    Moves to the pose to put down the last bottle of the recipe
    """
    def __init__(self, blackboard, _):
        super(MoveToBottlePose, self).__init__(blackboard)
        self.first_iteration = True
        self.repeat = False
        blackboard.arrived_at_bottle = False
        self.goal = MoveToTargetGoal()
        self.goal.target = ''
        self.goal.target_pose = blackboard.last_bottle_pose
        self.goal.look_at_target = False

    def perform(self, blackboard, reevaluate=False):
        if self.first_iteration or self.repeat:
            print('MoveToBottlePose')
            blackboard.move_action_client.send_goal(self.goal)
            self.first_iteration = False
            self.repeat = False

        # wait till action is completed
        if not blackboard.move_action_client.wait_for_result(rospy.Duration.from_sec(0.01)):
            return
        state = blackboard.move_action_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            blackboard.arrived_at_bottle = True
            self.pop()
        else:
            self.repeat = True

class GetNextBottle(AbstractActionElement):
    """
    Gets only pushed on the stack temporarily by the MakeCocktail decision.
    Only to signal that the next bottle is supposed to be brought and the stack to be emptied.
    """
    def perform(self, blackboard, reevaluate=False):
        print('getting next bottle')
        current_ingredient = blackboard.recipe.pop(0)
        blackboard.current_bottle = current_ingredient.keys()[0]
        blackboard.current_pour_time = current_ingredient.values()[0]
        blackboard.get_next_bottle = False
        blackboard.got_next_bottle = True

class Wait(AbstractActionElement):
    """
    Waits the amount of seconds given on init and pops
    """
    def __init__(self, blackboard, args=10):
        super(Wait, self).__init__(blackboard)
        self.resume_time = rospy.get_time() + args
	print('Waiting for %s seconds.'% args)

    def perform(self, connector, reevaluate=False):       
        if self.resume_time < rospy.get_time():
            self.pop()        
