#include <ros/ros.h>
#include <control_msgs/PointHeadAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_bartender_behavior/LookAt.h>

class LookAt
{
protected:
  ros::NodeHandle nh_;

public:
  LookAt() : ac_("/head_controller/point_head_action", true)
  {
    geometry_msgs::PointStamped named_target;
    named_target.header.frame_id = "torso_lift_link";
    named_target.point.x = 1.0;
    named_target.point.y = 0.0;
    named_target.point.z = 0.1;
    named_target_map_["forward"] = named_target;
    named_target.point.x = 0.0;
    named_target.point.y = 1.0;
    named_target_map_["left"] = named_target;
    named_target.point.y = -1.0;
    named_target_map_["right"] = named_target;
    named_target.point.x = 0.9;
    named_target.point.y = 0.0;
    named_target.point.z = -0.3;
    named_target_map_["down"] = named_target;

    current_goal_.pointing_frame = "xtion_optical_frame";
    current_goal_.pointing_axis.z = 1.0;
    current_goal_.min_duration = ros::Duration(0.5);
    current_goal_.max_velocity = 5.0;
    current_goal_.target = named_target_map_["forward"];

    look_at_server = nh_.advertiseService("head_controller/look_at_service", &LookAt::look_at_cb, this);
  }

  void run()
  {
    while(ros::ok())
    {
      current_goal_.target.header.stamp = ros::Time::now();
      ac_.sendGoal(current_goal_);
      ros::Duration(0.1).sleep();
    }
    ac_.cancelGoal();
  }

private:
  bool look_at_cb(tiago_bartender_behavior::LookAt::Request& req, tiago_bartender_behavior::LookAt::Response& res)
  {
    if(req.direction.empty())
    {
      current_goal_.target = req.target_point;
      return true;
    }
    if ( named_target_map_.find(req.direction) == named_target_map_.end() )
    {
      ROS_ERROR_STREAM("Requested direction not found in look_at_service");
    }
    else
    {
      current_goal_.target = named_target_map_[req.direction];
    }
    return true;
  }

  actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac_;
  control_msgs::PointHeadGoal current_goal_;
  std::map<std::string, geometry_msgs::PointStamped> named_target_map_;
  ros::ServiceServer look_at_server;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "look_at_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  LookAt la;
  la.run();
}
