#include <ros/ros.h>
#include <tiago_bartender_navigation/MoveToTargetAction.h>
#include <tiago_bartender_navigation/FindClosestTargetAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <control_msgs/PointHeadAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

class MoveToTarget
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tiago_bartender_navigation::MoveToTargetAction> as_mtt_;
  actionlib::SimpleActionServer<tiago_bartender_navigation::FindClosestTargetAction> as_fct_;
  std::string mtt_action_name_;
  std::string fct_action_name_;
  tiago_bartender_navigation::FindClosestTargetResult fct_res_;
  tiago_bartender_navigation::MoveToTargetResult mtt_res_;

public:
  MoveToTarget(std::string mtt_name, std::string fct_name) : as_mtt_(nh_, mtt_name, boost::bind(&MoveToTarget::executeMTT, this, _1), false), 
                                                 mtt_action_name_(mtt_name),
                                                 as_fct_(nh_, fct_name, boost::bind(&MoveToTarget::executeFCT, this, _1), false), 
                                                 fct_action_name_(fct_name),
                                                 ac_("move_base", true),
                                                 look_at_ac_("head_controller/look_at_action", true),
                                                 psi_()
  {
    ros::NodeHandle pn("~");

    pn.param("default_frame", default_frame_, std::string("map"));

    pn.param("line_segment_a_x", line_segment_a_x_, {});
    pn.param("line_segment_a_y", line_segment_a_y_, {});
    pn.param("line_segment_b_x", line_segment_b_x_, {});
    pn.param("line_segment_b_y", line_segment_b_y_, {});
    pn.param("ori_x", ori_x_, {});
    pn.param("ori_y", ori_y_, {});
    pn.param("ori_z", ori_z_, {});
    pn.param("ori_w", ori_w_, {});

    if(line_segment_a_x_.size() != line_segment_a_y_.size() || line_segment_a_y_.size() != line_segment_b_x_.size() || line_segment_b_x_.size() != line_segment_b_y_.size() || line_segment_b_y_.size() != ori_x_.size() || ori_y_.size() != ori_z_.size() || ori_z_.size() != ori_w_.size())
    {
      ROS_ERROR_STREAM("All vectors for the line segments must be the same size.");
    }

    //wait for the action server to come up
    while(!ac_.waitForServer(ros::Duration(5.0)) || !look_at_ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for action servers to come up");
    }

    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("move_to_target_marker", 0);

    as_mtt_.start();
    as_fct_.start();
  }

  void executeMTT(const tiago_bartender_navigation::MoveToTargetGoalConstPtr& goal)
  {
    geometry_msgs::PoseStamped target_pose;
    if(!goal->target.empty())
      target_pose = get_pose_from_id(goal->target);
    else
      target_pose = goal->target_pose;

    if(target_pose.header.frame_id.empty())
    {
      as_mtt_.setAborted();
      return;
    }

    // Point head to target
    if(goal->look_at_target)
    {
      control_msgs::PointHeadGoal ph_goal;
      ph_goal.target.point = goal->target_pose.pose.position;
      ph_goal.target.header = goal->target_pose.header;
      ph_goal.pointing_axis.z = 1.0;
      ph_goal.pointing_frame = "xtion_optical_frame";
      ph_goal.min_duration = ros::Duration(1);
      ph_goal.max_velocity = 1.0;
      look_at_ac_.sendGoal(ph_goal);
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

    move_to_target_pose(target_pose, false);

    marker.action = visualization_msgs::Marker::DELETE;
    vis_pub_.publish(marker);

    if(goal->look_at_target)
      look_at_ac_.cancelGoal();
  }


  void executeFCT(const tiago_bartender_navigation::FindClosestTargetGoalConstPtr &goal)
  {
    geometry_msgs::PoseStamped target_pose;
    target_pose = get_pose_from_type(goal->target_type, goal->target_pose);

    if(target_pose.header.frame_id.empty())
    {
      as_fct_.setAborted();
      return;
    }

    // Point head to target
    if(goal->look_at_target)
    {
      control_msgs::PointHeadGoal ph_goal;
      ph_goal.target.point = goal->target_pose.pose.position;
      ph_goal.target.header = goal->target_pose.header;
      ph_goal.pointing_axis.z = 1.0;
      ph_goal.pointing_frame = "xtion_optical_frame";
      ph_goal.min_duration = ros::Duration(1);
      ph_goal.max_velocity = 1.0;
      look_at_ac_.sendGoal(ph_goal);
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
    marker.text = "move to " + goal->target_type;
    vis_pub_.publish(marker);

    move_to_target_pose(target_pose, true);

    marker.action = visualization_msgs::Marker::DELETE;
    vis_pub_.publish(marker);

    if(goal->look_at_target)
      look_at_ac_.cancelGoal();

  }

  // set fct to true if the function is used by the find closest_target action server
  void move_to_target_pose(geometry_msgs::PoseStamped target_pose, bool fct)
  {
    geometry_msgs::PoseStamped matched_pose;
    matched_pose.header.frame_id = default_frame_;
    double min_distance = std::numeric_limits<double>::max();

    for(size_t i = 0; i < line_segment_a_x_.size(); i++)
    {
      double Ax = line_segment_a_x_[i];
      double Ay = line_segment_a_y_[i];
      double Bx = line_segment_b_x_[i];
      double By = line_segment_b_y_[i];
      double Cx = target_pose.pose.position.x;
      double Cy = target_pose.pose.position.y;

      double t=((Cx-Ax)*(Bx-Ax)+(Cy-Ay)*(By-Ay))/(std::pow(Bx-Ax, 2.0)+std::pow(By-Ay,2.0));
      double Dx=Ax+t*(Bx-Ax);
      double Dy=Ay+t*(By-Ay);
      double distance = std::abs(target_pose.pose.position.x - Dx) + std::abs(target_pose.pose.position.y - Dy);

      if(distance < min_distance)
      {
        min_distance = distance;
        matched_pose.pose.position.x = Dx;
        matched_pose.pose.position.y = Dy;
        matched_pose.pose.orientation.x = ori_x_[i];
        matched_pose.pose.orientation.y = ori_y_[i];
        matched_pose.pose.orientation.z = ori_z_[i];
        matched_pose.pose.orientation.w = ori_w_[i];
      }
    }

    move_base_msgs::MoveBaseGoal target;

    target.target_pose = matched_pose;

    ac_.sendGoal(target);

    while(!ac_.waitForResult(ros::Duration(0.5)) && ros::ok())
    {
      if(as_mtt_.isPreemptRequested() && !fct)
      {
        ROS_INFO("Moving to target was aborted.");
        ac_.cancelGoal();
        as_mtt_.setPreempted();
        return;
      }

      if(as_fct_.isPreemptRequested() && fct)
      {
        ROS_INFO("Finding cloest target was aborted.");
        ac_.cancelGoal();
        as_fct_.setPreempted();
        return;
      }
    }

    if(!fct)
    {
      mtt_res_.pose_result = matched_pose;
      mtt_res_.target_pose_result = target_pose;

      if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        as_mtt_.setSucceeded(mtt_res_);
      else
        as_mtt_.setAborted(mtt_res_);
    }

    if(fct)
    {
      fct_res_.pose_result = matched_pose;
      fct_res_.target_pose_result = target_pose;

      if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        as_fct_.setSucceeded(fct_res_);
      else
        as_fct_.setAborted(fct_res_);
    }

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

  geometry_msgs::PoseStamped get_pose_from_type(std::string type, geometry_msgs::PoseStamped pose)
  {
    geometry_msgs::PoseStamped matched_pose;
    std::map<std::string, moveit_msgs::CollisionObject> objects = psi_.getObjects();

    double min_distance = std::numeric_limits<double>::max();
    for(auto object : objects)
    {
      moveit_msgs::CollisionObject co = object.second;
      if (co.id.find(type) == std::string::npos)
      {
        continue;
      }
      geometry_msgs::PoseStamped target_pose;
      target_pose.header = co.header;

      if(!co.primitive_poses.empty())
      {
        target_pose.pose = co.primitive_poses.at(0);
      }
      else if(!co.mesh_poses.empty())
      {
        target_pose.pose = co.mesh_poses.at(0);
      }
      tf_listener_.transformPose(pose.header.frame_id, target_pose, target_pose);

      double distance = std::abs(target_pose.pose.position.x - pose.pose.position.x) + std::abs(target_pose.pose.position.y - pose.pose.position.y);
      if(distance < min_distance)
      {
        min_distance = distance;
        matched_pose = target_pose;
        fct_res_.target_id = co.id;
      }
    }

    return matched_pose;
  }

private:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  actionlib::SimpleActionClient<control_msgs::PointHeadAction> look_at_ac_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  tf::TransformListener tf_listener_;
  std::string default_frame_;

  std::vector<double> line_segment_a_x_;
  std::vector<double> line_segment_a_y_;
  std::vector<double> line_segment_b_x_;
  std::vector<double> line_segment_b_y_;
  std::vector<double> ori_x_;
  std::vector<double> ori_y_;
  std::vector<double> ori_z_;
  std::vector<double> ori_w_;

  ros::Publisher vis_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_target_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  MoveToTarget mtb("move_to_target", "find_closest_target");
  while(ros::ok())
  {

  }
}
