#include <ros/ros.h>
#include <control_msgs/PointHeadAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_bartender_msgs/LookAt.h>
#include <random>
#include <chrono>
#include <person_detection/PersonDetections.h>
#include <pal_common_msgs/DisableAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

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

    ros::NodeHandle bn("tiago_bartender/customer_zone");
    double la_radius_x;
    double la_radius_y;
    double la_radius_z;

    bn.param("frame", look_around_frame_, std::string("map"));
    bn.param("center_x", la_center_x_, 0.0);
    bn.param("center_y", la_center_y_, 0.0);
    bn.param("center_z", la_center_z_, 0.0);
    bn.param("size_x", la_radius_x, 0.0);
    bn.param("size_y", la_radius_y, 0.0);
    bn.param("size_z", la_radius_z, 0.0);
    bn.param("euler_z", look_around_rotation_, 0.0);
    bn.param("customer_distance_threshold", customer_distance_thresh_, 0.25);

    unif_x_ = std::uniform_real_distribution<double>(la_center_x_ - la_radius_x, la_center_x_ + la_radius_x);
    unif_y_ = std::uniform_real_distribution<double>(la_center_y_ - la_radius_y, la_center_y_ + la_radius_y);
    unif_z_ = std::uniform_real_distribution<double>(la_center_z_ - la_radius_z, la_center_z_ + la_radius_z);
    named_target.header.frame_id = "world";
    named_target.point.x = la_center_x_;
    named_target.point.y = la_center_y_;
    named_target.point.z = la_center_z_;
    named_target_map_["look_around"] = named_target;

    current_goal_.pointing_frame = "xtion_optical_frame";
    current_goal_.pointing_axis.z = 1.0;
    current_goal_.min_duration = ros::Duration(0.5);
    current_goal_.max_velocity = 5.0;
    current_goal_.target = named_target_map_["forward"];

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
    tiago_bartender_msgs::LookAt::Request req;
    tiago_bartender_msgs::LookAt::Response res;
    req.direction = "forward";
    look_at_cb(req, res);

    while(ros::ok())
    {
      if(current_target_name_ == "default")
      {
        ros::Duration(0.1).sleep();
        continue;
      }
      else if(current_target_name_ == "look_around")
      {
        if(ros::Duration(20.0) < (ros::Time::now() - look_around_start_))
        {
          look_around_start_ = ros::Time::now();
          Eigen::AngleAxis<double> aa(look_around_rotation_, Eigen::Vector3d(0,0,1));
          Eigen::Translation<double,3> origin(-la_center_x_, -la_center_y_, -la_center_z_);
          Eigen::Translation<double,3> back(la_center_x_, la_center_y_, la_center_z_);

          Eigen::Vector3d p1 = {unif_x_(rng_), unif_y_(rng_), unif_z_(rng_)};
          p1 = origin * p1;
          p1 = aa * p1;
          p1 = back * p1;
          current_goal_.target.point.x = p1(0);
          current_goal_.target.point.y = p1(1);
          current_goal_.target.point.z = p1(2);
          current_goal_.target.header.frame_id = look_around_frame_;
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
      std::map<std::string, moveit_msgs::CollisionObject> lookup= psi_.getObjects({req.direction});
      if(lookup.count(req.direction) == 0)
        ROS_ERROR_STREAM("Requested direction '" << req.direction << "'  not found in look_at_service");
      else {
        moveit_msgs::CollisionObject& co = lookup[req.direction];
        current_target_name_ = req.direction;
        current_goal_.target.header = co.header;
        if(co.primitive_poses.size() > 0){
          current_goal_.target.point = co.primitive_poses[0].position;
        }
        else if(co.mesh_poses.size() > 0){
          current_goal_.target.point = co.mesh_poses[0].position;
        }
        else {
          ROS_ERROR_STREAM("invalid collision object specified in look_at_service");
        }
      }
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
        current_goal_.target.point.z = unif_z_(rng_);
        current_goal_.target.header.frame_id = look_around_frame_;
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
  moveit::planning_interface::PlanningSceneInterface psi_;
  control_msgs::PointHeadGoal current_goal_;
  pal_common_msgs::DisableGoal disable_hm_;
  std::string current_target_name_;
  std::map<std::string, geometry_msgs::PointStamped> named_target_map_;
  ros::ServiceServer look_at_server;
  ros::Subscriber person_detection_sub_;
  std::uniform_real_distribution<double> unif_x_;
  std::uniform_real_distribution<double> unif_y_;
  std::uniform_real_distribution<double> unif_z_;
  double la_center_x_;
  double la_center_y_;
  double la_center_z_;
  std::mt19937_64 rng_;
  ros::Time look_around_start_;
  bool look_at_person_;
  geometry_msgs::PointStamped last_person_point_;
  double customer_distance_thresh_;
  std::string look_around_frame_;
  double look_around_rotation_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "look_at_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  LookAt la;
  la.run();
}
