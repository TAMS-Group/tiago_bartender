#include <ros/ros.h>
#include <functional>
#include <std_msgs/String.h>
#include <tiago_barkeeper_navigation/MoveToTargetAction.h>
#include <tiago_bartender_mtc/PickBottleAction.h>
#include <actionlib/client/simple_action_client.h>

struct StateMachine
{
  //void (StateMachine::*state)() = &StateMachine::state_init;
  std::function<void(StateMachine*)> state = &StateMachine::state_init;  

  std::string bottle_name;

  void state_init()
  {
    state = &StateMachine::state_watch;
  }
  
  void state_watch()
  {
    ROS_INFO("watching");
    ros::NodeHandle node;
    bool received = false;
    std::string name;
    auto selection = node.subscribe<std_msgs::String>("/selection", 1, boost::function<void(std_msgs::String)>(
      [&](std_msgs::String s) {
        ROS_INFO("selection");
        received = true;
        name = s.data;
      }
    ));
    while(true)
    {
      ros::Duration(0.1).sleep();
      if(received) break;
    }
    bottle_name = name;
    //state = &StateMachine::state_move_to_bottle;
    state = [name](StateMachine* m) { m->state_move_to_bottle(name); };
  }

  void state_move_to_bottle(const std::string& bottle_name)
  {
    move_to_target(bottle_name);
    state = [bottle_name](StateMachine* m) { m->state_grasp_bottle(bottle_name); };
  }
 
  void state_grasp_bottle(const std::string& bottle_name)
  {
    ROS_INFO("grasping");

    actionlib::SimpleActionClient<tiago_bartender_mtc::PickBottleAction> client("tiago_bottle_pick", true);
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

    state = &StateMachine::state_move_to_glass;
  }
  
  void state_move_to_glass()
  {
    ROS_INFO("moving to glass");
    ros::Duration(1).sleep();
    state = &StateMachine::state_pour;
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
    ros::Duration(1).sleep();
    state = &StateMachine::state_put_back_bottle;
  }

  void state_put_back_bottle()
  {
    ROS_INFO("putting bottle back");
    ros::Duration(1).sleep();
    state = &StateMachine::state_watch;
  }
 
  void move_to_target(const std::string& target_name)
  {
    ROS_INFO("moving to bottle %s", bottle_name.c_str());
    actionlib::SimpleActionClient<tiago_barkeeper_navigation::MoveToTargetAction> client("move_to_target", true);
    client.waitForServer();
    tiago_barkeeper_navigation::MoveToTargetGoal goal;
    goal.target = target_name;
    client.sendGoal(goal);
    while(!client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for move_to_target result.");
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully moved to bottle.");
    else
      ROS_ERROR("Moving to bottle aborted.");
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

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  StateMachine().run();
}
