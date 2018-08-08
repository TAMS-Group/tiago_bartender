#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_bartender_behavior/MenuOrderAction.h>
#include <tiago_bartender_navigation/MoveToTargetAction.h>
#include <control_msgs/PointHeadAction.h>
#include <std_msgs/String.h>
#include <tiago_bartender_speech/BartenderSpeechAction.h>

class MenuOrder
{
// setup for the action server
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tiago_bartender_behavior::MenuOrderAction> as_;
  std::string action_name_;

  tiago_bartender_behavior::MenuOrderFeedback feedback_;
  tiago_bartender_behavior::MenuOrderResult result_;

public:
  MenuOrder(std::string name) : as_(nh_, name, boost::bind(&MenuOrder::executeCB, this, _1), false),
                                action_name_(name),
                                move_to_target_client("move_to_target", true), 
                                point_head_client("head_controller/point_head_action", true),
                                look_at_client("head_controller/look_at_action", true),
                                speech_client("bartender_speech_action", true)
  {
    command_sub_ = nh_.subscribe("/command_cards/commands", 1000, &MenuOrder::command_callback, this);
    selection_sub_ = nh_.subscribe("/selection", 1000, &MenuOrder::selection_callback, this);
    move_to_target_client.waitForServer();
    point_head_client.waitForServer();
    look_at_client.waitForServer();
    as_.start();
  }

  void executeCB(const tiago_bartender_behavior::MenuOrderGoalConstPtr& goal)
  {
    // point head to look at customer
    control_msgs::PointHeadGoal ph_goal;
    ph_goal.target.header = goal->person_position.header;
    ph_goal.target.point = goal->person_position.point;
    ph_goal.pointing_frame = "xtion_optical_frame";
    ph_goal.pointing_axis.z = 1.0;
    ph_goal.min_duration = ros::Duration(0.5);
    ph_goal.max_velocity = 1.0;
    look_at_client.sendGoal(ph_goal);

    feedback_.state = "look at person";
    as_.publishFeedback(feedback_);

    // move in front of person
    tiago_bartender_navigation::MoveToTargetGoal mtt_goal;
    mtt_goal.target_pose.header = goal->person_position.header;
    mtt_goal.target_pose.pose.position = goal->person_position.point;
    mtt_goal.look_at_target = false;
    move_to_target_client.sendGoal(mtt_goal);
    while(!move_to_target_client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for move_to_target result.");
    if(move_to_target_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully moved in front of person.");
    else
    {
      ROS_ERROR("Moving to person aborted.");
      result_.error_message = "Moving to person failed";
      as_.setAborted(result_);
      return;
    }
    feedback_.state = "Moved in front of person";
    as_.publishFeedback(feedback_);


    // tell customer to order
    tiago_bartender_speech::BartenderSpeechGoal speech_goal;
    speech_goal.id = "ask_order";
    speech_client.sendGoal(speech_goal);
    speech_client.waitForResult();

    // stop looking at customer
    look_at_client.cancelGoal();
    ros::Duration(0.2).sleep();

    // Point head down to look on menu
    selection_ = "";
    ph_goal.target.header.frame_id = "base_link";
    ph_goal.target.point.x = 0.3;
    ph_goal.target.point.y = 0.0;
    ph_goal.target.point.z = 0.5;
    ph_goal.pointing_frame = "xtion_optical_frame";
    ph_goal.pointing_axis.z = 1.0;
    ph_goal.min_duration = ros::Duration(0.5);
    ph_goal.max_velocity = 1.0;
    point_head_client.sendGoal(ph_goal);
    while(!point_head_client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for point_head result.");
    if(point_head_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully pointed head.");
    else
    {
      ROS_ERROR("Pointing head to menu failed.");
    }
    feedback_.state = "Point head to menu";
    as_.publishFeedback(feedback_);
    std::string selection = "";
    ros::Time begin = ros::Time::now();
    while(selection.empty() || (ros::Time::now() - begin) > ros::Duration(60.0))
    {
      if(selection_time_ > begin)
      {
        selection = selection_;
      }
    }

    if(selection.empty())
    {
      result_.selection = selection;
      result_.error_message = "no selection";
      as_.setAborted(result_);
      return;
    }

    // Point head to look at the customer
    ph_goal.target.header = goal->person_position.header;
    ph_goal.target.point = goal->person_position.point;
    ph_goal.pointing_frame = "xtion_optical_frame";
    ph_goal.pointing_axis.z = 1.0;
    ph_goal.min_duration = ros::Duration(0.5);
    ph_goal.max_velocity = 1.0;
    point_head_client.sendGoal(ph_goal);
    while(!point_head_client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for point_head result.");
    if(point_head_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully pointed head.");
    else
    {
      ROS_ERROR("Pointing head to menu failed.");
    }
    feedback_.state = "Point head to customer";
    as_.publishFeedback(feedback_);

    // todo: audio output stating order and giving opportunity to cancel via command card

    result_.error_message = "";
    result_.selection = selection;
    as_.setSucceeded(result_);
  }

private:
  actionlib::SimpleActionClient<tiago_bartender_navigation::MoveToTargetAction> move_to_target_client;
  actionlib::SimpleActionClient<control_msgs::PointHeadAction> point_head_client;
  actionlib::SimpleActionClient<control_msgs::PointHeadAction> look_at_client;
  actionlib::SimpleActionClient<tiago_bartender_speech::BartenderSpeechAction> speech_client;

  void command_callback(const std_msgs::String::ConstPtr& msg)
  {
    command_time_ = ros::Time::now();
    command_ = msg->data;
  }


  void selection_callback(const std_msgs::String::ConstPtr& msg)
  {
    selection_time_ = ros::Time::now();
    selection_ = msg->data;
  }

  ros::Subscriber command_sub_;
  ros::Subscriber selection_sub_;

  std::string command_;
  std::string selection_;
  ros::Time command_time_;
  ros::Time selection_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "menu_order_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  MenuOrder menu_order("take_order_action");

  while(ros::ok())
  {}
}
