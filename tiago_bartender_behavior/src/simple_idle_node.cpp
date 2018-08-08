#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>

struct Target
{
  move_base_msgs::MoveBaseGoal move_target;
  trajectory_msgs::JointTrajectory head_target;
};

class SimpleIdle
{
public:
  SimpleIdle(): enabled_(false), ac_("move_base", true)
  {
    switch_service_ = nh_.advertiseService("switch_simple_idle", &SimpleIdle::switch_idle, this);
    head_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1000);
    Target target;
    target.move_target.target_pose.header.frame_id = "map";
    target.move_target.target_pose.pose.position.x = -0.75;
    target.move_target.target_pose.pose.position.y = -1.5;
    target.move_target.target_pose.pose.orientation.w = 0.7071;
    target.move_target.target_pose.pose.orientation.z = -0.7071;
    target.head_target.joint_names.push_back("head_1_joint");
    target.head_target.joint_names.push_back("head_2_joint");
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.push_back(-1.29);
    jtp.positions.push_back(0.0);
    jtp.time_from_start = ros::Duration(0.5);
    target.head_target.points.push_back(jtp);
    targets_.push_back(target);
    target.move_target.target_pose.pose.orientation.z = 0.7071;
    target.move_target.target_pose.pose.position.y = 1.5;
    target.head_target.points.at(0).positions.at(0) = 1.29;
    targets_.push_back(target);

    while(!ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for move_base action server to come up.");
    }
  }

  void run()
  {
    auto itr = targets_.begin();

    while(ros::ok())
    {
      if(!enabled_)
        continue;

      if(itr == targets_.end())
        itr = targets_.begin();

      itr->head_target.header.stamp = ros::Time::now() + ros::Duration(0.1);
      head_command_pub_.publish(itr->head_target);

      ac_.sendGoal(itr->move_target);
      while(!ac_.waitForResult(ros::Duration(0.1)) && ros::ok())
      {
        if(!enabled_)
        {
          ac_.cancelGoal();
        }
      }

      itr++;
    }
  }

private:
  bool switch_idle(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    enabled_ = !enabled_;
    return true;
  }

  ros::NodeHandle nh_;
  ros::ServiceServer switch_service_;
  ros::Publisher head_command_pub_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  bool enabled_;
  std::vector<Target> targets_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_idle_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  SimpleIdle simple_idle;
  simple_idle.run();
}
