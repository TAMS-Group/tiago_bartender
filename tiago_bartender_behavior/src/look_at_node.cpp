#include <ros/ros.h>
#include <control_msgs/PointHeadAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_bartender_msgs/LookAt.h>
#include <random>
#include <chrono>
#include <person_detection/PersonDetections.h>
#include <pal_common_msgs/DisableAction.h>

class LookAt
{
protected:
  ros::NodeHandle nh_;

public:
  LookAt() : ac_("/head_controller/point_head_action", true),
             hm_ac_("pal_head_manager/disable", true)
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
    pn.param("customer_distance_threshold", customer_distance_thresh_, 0.25);

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

    tiago_bartender_msgs::LookAt::Request req;
    tiago_bartender_msgs::LookAt::Response res;
    req.direction = "forward";
    look_at_cb(req, res);

    disable_hm_.duration = 0.0;

    uint64_t time_seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(time_seed & 0xffffffff), uint32_t(time_seed>>32)};
    rng_.seed(ss);

    look_at_server = nh_.advertiseService("head_controller/look_at_service", &LookAt::look_at_cb, this);
    person_detection_sub_ = nh_.subscribe("person_detection/person_detections", 1000, &LookAt::person_detection_cb, this);

    ac_.waitForServer();
    hm_ac_.waitForServer();
  }

  void run()
  {
    while(ros::ok())
    {
      if(current_target_name_ == "default")
        return;
      else if(current_target_name_ == "look_around")
      {
        if(ros::Duration(20.0) < (ros::Time::now() - look_around_start_))
        {
          look_around_start_ = ros::Time::now();
          current_goal_.target.point.x = unif_x_(rng_);
          current_goal_.target.point.y = unif_y_(rng_);
        }
      }
      else if(current_target_name_ == "customer")
      {
        current_goal_.target = last_person_point_;
      }
      current_goal_.target.header.stamp = ros::Time::now();
      ac_.sendGoal(current_goal_);
      ros::Duration(0.1).sleep();
    }
    ac_.cancelGoal();
  }

private:
  bool look_at_cb(tiago_bartender_msgs::LookAt::Request& req, tiago_bartender_msgs::LookAt::Response& res)
  {
    if(req.direction == "default")
    {
      current_target_name_ = req.direction;
      ac_.cancelGoal();
      hm_ac_.cancelGoal();
      return true;
    }
    else
      hm_ac_.sendGoal(disable_hm_);

    if(req.direction.empty())
    {
      current_goal_.target = req.target_point;
      current_target_name_ = "";
    }
    else if(req.direction == "customer")
    {
      current_goal_.target = req.target_point;
      last_person_point_ = current_goal_.target;
      current_target_name_ = req.direction;
    }
    else if ( named_target_map_.find(req.direction) == named_target_map_.end() )
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

  void person_detection_cb(const person_detection::PersonDetections::ConstPtr& msg)
  {
    double min_distance = std::numeric_limits<double>::max();
    for(auto pd : msg->detections)
    {
      double distance = std::sqrt(std::pow(last_person_point_.point.x - pd.position.x, 2.0) +
                                  std::pow(last_person_point_.point.y - pd.position.y, 2.0) +
                                  std::pow(last_person_point_.point.z - pd.position.z, 2.0));
      if(distance < min_distance && distance < customer_distance_thresh_)
        last_person_point_.point = pd.position;
    }
  }

  actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac_;
  actionlib::SimpleActionClient<pal_common_msgs::DisableAction> hm_ac_;
  control_msgs::PointHeadGoal current_goal_;
  pal_common_msgs::DisableGoal disable_hm_;
  std::string current_target_name_;
  std::map<std::string, geometry_msgs::PointStamped> named_target_map_;
  ros::ServiceServer look_at_server;
  ros::Subscriber person_detection_sub_;
  std::uniform_real_distribution<double> unif_x_;
  std::uniform_real_distribution<double> unif_y_;
  std::mt19937_64 rng_;
  ros::Time look_around_start_;
  bool look_at_person_;
  geometry_msgs::PointStamped last_person_point_;
  double customer_distance_thresh_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "look_at_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  LookAt la;
  la.run();
}
