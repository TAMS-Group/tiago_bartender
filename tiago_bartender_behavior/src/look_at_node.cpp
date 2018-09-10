#include <ros/ros.h>
#include <control_msgs/PointHeadAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_bartender_behavior/LookAt.h>
#include <random>
#include <chrono>

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
    named_target.point.z = 0.3;
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

    ros::NodeHandle pn("~");
    double la_center_x;
    double la_center_y;
    double la_radius_x;
    double la_radius_y;
    double la_z_value;
    pn.param("look_around_center_x", la_center_x, 0.0);
    pn.param("look_around_center_y", la_center_y, 0.0);
    pn.param("look_around_radius_x", la_radius_x, 0.0);
    pn.param("look_around_radius_y", la_radius_y, 0.0);
    pn.param("look_around_z_value", la_z_value, 1.8);

    unif_x_ = std::uniform_real_distribution<double>(la_center_x - la_radius_x, la_center_x + la_radius_x);
    unif_y_ = std::uniform_real_distribution<double>(la_center_y - la_radius_y, la_center_y + la_radius_y);
    named_target.header.frame_id = "world";
    named_target.point.x = la_center_x;
    named_target.point.y = la_center_y;
    named_target.point.z = la_z_value;
    named_target_map_["look_around"] = named_target;

    current_goal_.pointing_frame = "xtion_optical_frame";
    current_goal_.pointing_axis.z = 1.0;
    current_goal_.min_duration = ros::Duration(0.5);
    current_goal_.max_velocity = 5.0;
    current_goal_.target = named_target_map_["forward"];

    uint64_t time_seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(time_seed & 0xffffffff), uint32_t(time_seed>>32)};
    rng_.seed(ss);

    look_at_server = nh_.advertiseService("head_controller/look_at_service", &LookAt::look_at_cb, this);
  }

  void run()
  {
    while(ros::ok())
    {
      if(current_target_name_ == "look_around")
      {
        if(ros::Duration(20.0) < (ros::Time::now() - look_around_start_))
        {
          look_around_start_ = ros::Time::now();
          current_goal_.target.point.x = unif_x_(rng_);
          current_goal_.target.point.y = unif_y_(rng_);
        }
      }
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
      current_target_name_ = "";
      return true;
    }
    if ( named_target_map_.find(req.direction) == named_target_map_.end() )
    {
      ROS_ERROR_STREAM("Requested direction not found in look_at_service");
    }
    else
    {
      current_goal_.target = named_target_map_[req.direction];
      current_target_name_ = req.direction;
      if(current_target_name_ == "look_around")
      {
        look_around_start_ = ros::Time::now();
        current_goal_.target.point.x = unif_x_(rng_);
        current_goal_.target.point.y = unif_y_(rng_);
      }
    }
    return true;
  }

  actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac_;
  control_msgs::PointHeadGoal current_goal_;
  std::string current_target_name_;
  std::map<std::string, geometry_msgs::PointStamped> named_target_map_;
  ros::ServiceServer look_at_server;
  std::uniform_real_distribution<double> unif_x_;
  std::uniform_real_distribution<double> unif_y_;
  std::mt19937_64 rng_;
  ros::Time look_around_start_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "look_at_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  LookAt la;
  la.run();
}
