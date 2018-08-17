#include <ros/ros.h>
#include <functional>
#include <std_msgs/String.h>
#include <tiago_bartender_navigation/MoveToTargetAction.h>
#include <tiago_bartender_navigation/FindClosestTargetAction.h>
#include <tiago_bartender_mtc/PickBottleAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <tiago_bartender_behavior/MenuOrderAction.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

struct StateMachine
{
  std::function<void(StateMachine*)> state = &StateMachine::state_init;  

  std::string bottle_name;

  void state_init()
  {
    ROS_INFO("idling");
    extend_torso();

    publish_marker("state_init");
    ros::NodeHandle node;

    // start idle behavior
    ros::ServiceClient client = node.serviceClient<std_srvs::Empty>("switch_simple_idle");
    std_srvs::Empty srv;
    if (!client.call(srv))
    {
      ROS_ERROR("Failed to call service switch_simple_idle");
      state = &StateMachine::state_init;
      return;
    }

    // Wait for a person to be detected
    bool received = false;
    geometry_msgs::PointStamped person_position;
    auto selection = node.subscribe<geometry_msgs::PointStamped>("/person_detection", 1, boost::function<void(geometry_msgs::PointStamped)>(
      [&](geometry_msgs::PointStamped p) {
        ROS_INFO("person pose");
        received = true;
        person_position = p;
      }
    ));
    while(true)
    {
      ros::Duration(0.1).sleep();
      if(received) break;
    }

    // stop idle behavior
    if (!client.call(srv))
    {
      ROS_ERROR("Failed to call service switch_simple_idle");
      state = &StateMachine::state_init;
      return;
    }
    sleep(0.1);

    state = [person_position](StateMachine* m) { m->state_take_order(person_position); };
  }

  void state_take_order(const geometry_msgs::PointStamped person_position)
  {
    ROS_INFO("Taking order");
    publish_marker("state_take_order");
    actionlib::SimpleActionClient<tiago_bartender_behavior::MenuOrderAction> client("take_order_action", true);
    client.waitForServer();
    tiago_bartender_behavior::MenuOrderGoal goal;
    goal.person_position = person_position;
    client.sendGoal(goal);
    while(!client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for menu order result.");
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully took order.");
    else
    {
      ROS_ERROR("Taking order aborted.");
      state = [person_position](StateMachine* m) { m->state_take_order(person_position); };
      return;
    }

    auto result = client.getResult();
    std::string bottle_name = result->selection;

    last_person_position_ = person_position;

    state = [bottle_name](StateMachine* m) { m->state_move_to_bottle(bottle_name); };
  }

  void state_move_to_bottle(const std::string& bottle_name)
  {
    publish_marker("state_move_to_bottle");
    bool success = move_to_target(bottle_name, true);
    if(success)
      state = [bottle_name](StateMachine* m) { m->state_update_scene(bottle_name); };
    else
      state = [bottle_name](StateMachine* m) { m->state_move_to_bottle(bottle_name); };
  }

  void state_update_scene(const std::string& bottle_name)
  {
    // todo: replace this place holder with actual update method
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<std_srvs::Empty>("dummy_planning_scene/update_planning_scene");
    std_srvs::Empty srv;
    if (!client.call(srv))
    {
      ROS_ERROR("Failed to call service update_planning_scene");
      state = [bottle_name](StateMachine* m) { m->state_update_scene(bottle_name); };
      return;
    }
    state = [bottle_name](StateMachine* m) { m->state_grasp_bottle(bottle_name); };
  }
 
  void state_grasp_bottle(const std::string& bottle_name)
  {
    ROS_INFO("grasping");
    publish_marker("state_grasp_bottle");

    /*actionlib::SimpleActionClient<tiago_bartender_mtc::PickBottleAction> client("tiago_bottle_pick", true);
    client.waitForServer();
    tiago_bartender_mtc::PickBottleGoal goal;
    goal.bottle_id = bottle_name;
    client.sendGoal(goal);
    while(!client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for bottle grasp result.");
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully grasped to bottle.");
    else
      ROS_ERROR("Grasping bottle aborted.");
    */

    state = &StateMachine::state_move_to_glass;
  }
  
  void state_move_to_glass()
  {
    ROS_INFO("moving to glass");
    publish_marker("state_move_to_glass");
    geometry_msgs::PoseStamped person_pose;
    person_pose.header = last_person_position_.header;
    person_pose.pose.position = last_person_position_.point;
    std::string target_id_result;
    bool success = find_closest_target("glass", person_pose, true, target_id_result);
    if(success)
      state = [target_id_result](StateMachine* m) { m->state_pour(target_id_result); };
    else
      state = &StateMachine::state_move_to_glass;
  }

  void state_pour(const std::string& glass_id)
  {
    ROS_INFO("pouring");
    publish_marker("state_pour");
    ros::Duration(1).sleep();
    state = &StateMachine::state_move_back_to_shelf;
  }

  void state_move_back_to_shelf()
  {
    ROS_INFO("moving back to shelf");
    publish_marker("state_move_back_to_shelf");
    bool success = move_to_pose(last_object_target_pose_, true);
    if(success)
      state = &StateMachine::state_put_back_bottle;
    else
      state = &StateMachine::state_move_back_to_shelf;
  }

  void state_put_back_bottle()
  {
    ROS_INFO("putting bottle back");
    publish_marker("state_put_back_bottle");
    ros::Duration(1).sleep();
    state = &StateMachine::state_init;
  }
 
  bool move_to_target(const std::string& target_name, bool look_at_target)
  {
    ROS_INFO("moving to target %s", target_name.c_str());
    actionlib::SimpleActionClient<tiago_bartender_navigation::MoveToTargetAction> client("move_to_target", true);
    client.waitForServer();
    tiago_bartender_navigation::MoveToTargetGoal goal;
    goal.target = target_name;
    goal.look_at_target = look_at_target;
    client.sendGoal(goal);
    while(!client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for move_to_target result.");
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Successfully moved to target.");
      last_object_target_pose_ = client.getResult()->target_pose_result;
      return true;
    }
    else
    {
      ROS_ERROR("Moving to target aborted.");
      return false;
    }
  }

  bool find_closest_target(const std::string& type_name, const geometry_msgs::PoseStamped& pose, const bool look_at_target, std::string& target_id_result)
  {
    ROS_INFO("Finding closest target of type %s", type_name.c_str());
    actionlib::SimpleActionClient<tiago_bartender_navigation::FindClosestTargetAction> client("find_closest_target", true);
    client.waitForServer();
    tiago_bartender_navigation::FindClosestTargetGoal goal;
    goal.target_type = type_name;
    goal.target_pose = pose;
    goal.look_at_target = look_at_target;
    client.sendGoal(goal);
    while(!client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for find_closest_target result.");
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Successfully found and moved to target.");
      target_id_result = client.getResult()->target_id;
      return true;
    }
    else
    {
      ROS_ERROR("Find closest target aborted.");
      return false;
    }
  }

  bool move_to_pose(const geometry_msgs::PoseStamped pose, bool look_at_target)
  {
    ROS_INFO("moving to pose.");
    actionlib::SimpleActionClient<tiago_bartender_navigation::MoveToTargetAction> client("move_to_target", true);
    client.waitForServer();
    tiago_bartender_navigation::MoveToTargetGoal goal;
    goal.target_pose = pose;
    goal.look_at_target = look_at_target;
    client.sendGoal(goal);
    while(!client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for move_to_target result.");
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Successfully moved to target.");
      return true;
    }
    else
    {
      ROS_ERROR("Moving to target aborted.");
      return false;
    }
  }

  void publish_marker(const std::string& text)
  {
    marker_.text = text;
    marker_pub_.publish(marker_);
  }

  void init()
  {
    ROS_INFO("init");
    // prepare the marker for use later on
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("bartender_state_marker", 0);
    torso_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
    ros::Duration(1.0).sleep();

    marker_.header.frame_id = "base_footprint";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = 0.25;
    marker_.pose.position.y = 0.25;
    marker_.pose.position.z = 2.0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.25;
    marker_.scale.y = 0.25;
    marker_.scale.z = 0.25;
    marker_.color.a = 1.0;
    marker_.color.r = 1.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.0;

  }

  void extend_torso()
  {
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("torso_controller/follow_joint_trajectory", true);
    client.waitForServer();
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory torso_command;
    torso_command.joint_names.push_back("torso_lift_joint");
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.push_back(0.35);
    jtp.time_from_start = ros::Duration(0.5);
    torso_command.points.push_back(jtp);
    torso_command_pub_.publish(torso_command);
    goal.trajectory = torso_command;

    client.sendGoal(goal);
    while(!client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting to extend torso.");
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully extended torso.");
  }
  
  void run()
  {
    init();
    while(true)
    {
      state(this);
    }
  }

  geometry_msgs::PointStamped last_person_position_;
  // pose of the object the robot moved to most recently
  geometry_msgs::PoseStamped last_object_target_pose_;
  ros::Publisher marker_pub_;
  ros::Publisher torso_command_pub_;
  visualization_msgs::Marker marker_;
  ros::NodeHandle nh_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  StateMachine().run();
}
