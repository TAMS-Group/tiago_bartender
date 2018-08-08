#include <ros/ros.h>
#include <control_msgs/PointHeadAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

class LookAt
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::PointHeadAction> as_;
  std::string action_name_;

public:
  LookAt(std::string action_name) : as_(nh_, action_name, boost::bind(&LookAt::executeCB, this, _1), false),
                                    action_name_(action_name),
                                    ac_("/head_controller/point_head_action", true)
  {
    as_.start();
  }

private:
  void executeCB(const control_msgs::PointHeadGoalConstPtr& goal)
  {
    while(!as_.isPreemptRequested())
    {
      ac_.sendGoal(*goal);
      ros::Duration(0.1).sleep();
    }
    ac_.cancelGoal();
    as_.setPreempted();
  }
  actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "look_at_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  LookAt la("/head_controller/look_at_action");
  while(ros::ok())
  {

  }
}
