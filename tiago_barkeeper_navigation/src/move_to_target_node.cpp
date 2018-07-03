#include <ros/ros.h>
#include <tiago_barkeeper_navigation/MoveToTargetAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

class MoveToTarget
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tiago_barkeeper_navigation::MoveToTargetAction> as_;
  std::string action_name_;

public:
  MoveToTarget(std::string name) : as_(nh_, name, boost::bind(&MoveToTarget::executeCB, this, _1), false), 
                                   action_name_(name),
                                   ac_("move_base", true),
                                   psi_()
  {
    ros::NodeHandle pn("~");

    pn.param("default_frame", default_frame_, std::string("map"));

    std::vector<double> pos_x;
    std::vector<double> pos_y;
    std::vector<double> ori_x;
    std::vector<double> ori_y;
    std::vector<double> ori_z;
    std::vector<double> ori_w;
    pn.param("pos_x", pos_x, {});
    pn.param("pos_y", pos_y, {});
    pn.param("ori_x", ori_x, {});
    pn.param("ori_y", ori_y, {});
    pn.param("ori_z", ori_z, {});
    pn.param("ori_w", ori_w, {});

    if(pos_x.size() != pos_y.size() || pos_y.size() != ori_x.size() || ori_y.size() != ori_z.size() || ori_z.size() != ori_w.size())
    {
      ROS_ERROR_STREAM("All vectors for the predefined poses must be the same size.");
    }
    else
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = default_frame_;
      pose.pose.position.z = 0.0;
      for(int i=0; i < pos_x.size(); i++)
      {
        pose.pose.position.x = pos_x[i];
        pose.pose.position.y = pos_y[i];
        pose.pose.orientation.x = ori_x[i];
        pose.pose.orientation.y = ori_y[i];
        pose.pose.orientation.z = ori_z[i];
        pose.pose.orientation.w = ori_w[i];
        predefined_poses_.push_back(pose);
      }
    }

    //wait for the action server to come up
    while(!ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("move_to_target_marker", 0);

    as_.start();
  }

  void executeCB(const tiago_barkeeper_navigation::MoveToTargetGoalConstPtr &goal)
  {
    geometry_msgs::PoseStamped target_pose = get_pose_from_id(goal->target);
    if(target_pose.header.frame_id.empty())
    {
      as_.setAborted();
      return;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 1.75;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "move to " + goal->target;
    vis_pub_.publish(marker);

    geometry_msgs::PoseStamped matched_pose;
    double min_distance = std::numeric_limits<double>::max();
    for(geometry_msgs::PoseStamped pose : predefined_poses_)
    {
      double distance = std::abs(target_pose.pose.position.x - pose.pose.position.x) + std::abs(target_pose.pose.position.y - pose.pose.position.y);
      if(distance < min_distance)
      {
        min_distance = distance;
        matched_pose = pose;
      }
    }

    move_base_msgs::MoveBaseGoal target;

    //we'll send a goal to the robot to move 1 meter forward
    target.target_pose = matched_pose;

    ac_.sendGoal(target);

    while(!ac_.waitForResult(ros::Duration(0.5)) && ros::ok())
    {
      if(as_.isPreemptRequested())
      {
        ROS_INFO("Moving to target was aborted.");
        ac_.cancelGoal();
        as_.setPreempted();
      }
    }

    if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      as_.setSucceeded();
    else
      as_.setAborted();

    marker.action = visualization_msgs::Marker::DELETE;
    vis_pub_.publish(marker);
  }

  geometry_msgs::PoseStamped get_pose_from_id(std::string target_id)
  {
    geometry_msgs::PoseStamped pose;
    std::map<std::string, moveit_msgs::CollisionObject> objects = psi_.getObjects(std::vector<std::string>({target_id}));
    if(objects.empty())
    {
      ROS_ERROR("Target object not in planning scene");
      return pose;
    }
    moveit_msgs::CollisionObject object = objects[target_id];
    pose.header = object.header;

    if(!object.primitive_poses.empty())
    {
      pose.pose = object.primitive_poses.at(0);
    }
    else if(!object.mesh_poses.empty())
    {
      pose.pose = object.mesh_poses.at(0);
    }
    tf_listener_.transformPose(default_frame_, pose, pose);
    return pose;
  }

private:
  std::vector<geometry_msgs::PoseStamped> predefined_poses_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  tf::TransformListener tf_listener_;
  std::string default_frame_;

  ros::Publisher vis_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_target_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  MoveToTarget mtb("move_to_target");
  while(ros::ok())
  {

  }
}
