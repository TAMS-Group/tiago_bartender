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

struct StateMachine
{
  //void (StateMachine::*state)() = &StateMachine::state_init;
  std::function<void(StateMachine*)> state = &StateMachine::state_init;  

  std::string bottle_name;

  void state_init()
  {
    ROS_INFO("idling");
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
    bool success = move_to_target(bottle_name, true);
    if(success)
      state = [bottle_name](StateMachine* m) { m->state_grasp_bottle(bottle_name); };
    else
      state = [bottle_name](StateMachine* m) { m->state_move_to_bottle(bottle_name); };
  }
 
  void state_grasp_bottle(const std::string& bottle_name)
  {
    ROS_INFO("grasping");

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
    geometry_msgs::PoseStamped person_pose;
    person_pose.header = last_person_position_.header;
    person_pose.pose.position = last_person_position_.point;
    bool success = find_closest_target("glass", person_pose, true);
    if(success)
      state = &StateMachine::state_pour;
    else
      state = &StateMachine::state_move_to_glass;
  }

  void state_pour()
  {
    ROS_INFO("pouring");
    ros::Duration(1).sleep();
    state = &StateMachine::state_move_back_to_shelf;
  }

  void state_move_back_to_shelf()
  {
    ROS_INFO("moving back to shelf");
    bool success = move_to_pose(last_robot_target_pose_);
    if(success)
      state = &StateMachine::state_put_back_bottle;
    else
      state = &StateMachine::state_move_back_to_shelf;
  }

  void state_put_back_bottle()
  {
    ROS_INFO("putting bottle back");
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
      last_robot_target_pose_ = client.getResult()->pose_result;
      return true;
    }
    else
    {
      ROS_ERROR("Moving to target aborted.");
      return false;
    }
  }

  bool find_closest_target(const std::string& type_name, const geometry_msgs::PoseStamped& pose, bool look_at_target)
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
      return true;
    }
    else
    {
      ROS_ERROR("Find closest target aborted.");
      return false;
    }
  }

  bool move_to_pose(const geometry_msgs::PoseStamped pose)
  {
    ROS_INFO("moving to pose.");
    actionlib::SimpleActionClient<tiago_bartender_navigation::MoveToTargetAction> client("move_to_target", true);
    client.waitForServer();
    tiago_bartender_navigation::MoveToTargetGoal goal;
    goal.target_pose = pose;
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
  
  void run()
  {
    while(true)
    {
      state(this);
      //(this->*state)();
      //ROS_INFO("state %i", (int)(size_t)(void*)state);
      //state = s;
    }
  }

  geometry_msgs::PointStamped last_person_position_;
  // pose the robot was in when it last grasped the bottle
  geometry_msgs::PoseStamped last_robot_target_pose_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  StateMachine().run();
}
