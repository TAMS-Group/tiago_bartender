#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_bartender_behavior/MenuOrderAction.h>
#include <tiago_bartender_navigation/MoveToTargetAction.h>
#include <control_msgs/PointHeadAction.h>
#include <std_msgs/String.h>
#include <tiago_bartender_speech/BartenderSpeechAction.h>
#include <tiago_bartender_behavior/LookAt.h>

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
                                speech_client("bartender_speech_action", true)
  {
    command_sub_ = nh_.subscribe("/command_cards/commands", 1000, &MenuOrder::command_callback, this);
    selection_sub_ = nh_.subscribe("/selection", 1000, &MenuOrder::selection_callback, this);
    look_at_client_ = nh_.serviceClient<tiago_bartender_behavior::LookAt>("head_controller/look_at_service");
    move_to_target_client.waitForServer();
    as_.start();
  }

  void executeCB(const tiago_bartender_behavior::MenuOrderGoalConstPtr& goal)
  {
    // tell customer to order
    tiago_bartender_speech::BartenderSpeechGoal speech_goal;
    speech_goal.id = "ask_order";
    speech_client.sendGoal(speech_goal);
    speech_client.waitForResult();

    // Point head down to look on menu
    tiago_bartender_behavior::LookAt srv;
    srv.request.direction = "down";
    if (!look_at_client_.call(srv))
    {
      ROS_ERROR("Failed to call look_at_service");
    }
    feedback_.state = "Point head to menu";
    as_.publishFeedback(feedback_);

    // Wait for order
    selection_ = "";
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
    srv.request.direction = "";
    srv.request.target_point.header = goal->person_position.header;
    srv.request.target_point.point = goal->person_position.point;
    if (!look_at_client_.call(srv))
    {
      ROS_ERROR("Failed to call look_at_service");
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
  ros::ServiceClient look_at_client_;

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
