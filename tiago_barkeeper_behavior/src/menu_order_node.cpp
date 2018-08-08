#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_barkeeper_behavior/MenuOrderAction.h>
#include <tiago_barkeeper_navigation/MoveToTargetAction.h>
#include <control_msgs/PointHeadAction.h>
#include <std_msgs/String.h>
#include <tiago_bartender_speech/FilePath.h>

class MenuOrder
{
// setup for the action server
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tiago_barkeeper_behavior::MenuOrderAction> as_;
  std::string action_name_;

  tiago_barkeeper_behavior::MenuOrderFeedback feedback_;
  tiago_barkeeper_behavior::MenuOrderResult result_;

public:
  MenuOrder(std::string name) : as_(nh_, name, boost::bind(&MenuOrder::executeCB, this, _1), false),
                                action_name_(name),
                                move_to_target_client("move_to_target", true), 
                                point_head_client("head_controller/point_head_action", true)

  {
    command_sub_ = nh_.subscribe("/command_cards/commands", 1000, &MenuOrder::command_callback, this);
    selection_sub_ = nh_.subscribe("/selection", 1000, &MenuOrder::selection_callback, this);
    sound_client_ = nh_.serviceClient<tiago_bartender_speech::FilePath>("play_audio_file");
    move_to_target_client.waitForServer();
    point_head_client.waitForServer();
    as_.start();
  }

  void executeCB(const tiago_barkeeper_behavior::MenuOrderGoalConstPtr& goal)
  {
    // move in front of person
    tiago_barkeeper_navigation::MoveToTargetGoal mtt_goal;
    mtt_goal.target_pose.header = goal->person_position.header;
    mtt_goal.target_pose.pose.position = goal->person_position.point;
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

    // point head to look at customer
    control_msgs::PointHeadGoal ph_goal;
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
      ROS_ERROR("Pointing head failed.");
    }
    feedback_.state = "Point head to person";
    as_.publishFeedback(feedback_);

    // tell customer to order
    tiago_bartender_speech::FilePath srv;
    srv.request.file_path = "ask_order.wav";
    if (!sound_client_.call(srv))
    {
      ROS_ERROR("Failed to call sound service");
    }

    // Point head down to look on menu
    // ToDo: confirm that menu was found or search for it otherwise?
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
    result_.error_message = "";
    result_.selection = selection;
    as_.setSucceeded(result_);
  }

private:
  actionlib::SimpleActionClient<tiago_barkeeper_navigation::MoveToTargetAction> move_to_target_client;
  actionlib::SimpleActionClient<control_msgs::PointHeadAction> point_head_client;

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
  ros::ServiceClient sound_client_;

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
