#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tiago_bartender_msgs/FindClosestTargetAction.h>
#include <tiago_bartender_msgs/LookAt.h>
#include <tiago_bartender_msgs/MoveToTargetAction.h>
#include <visualization_msgs/Marker.h>

class MoveToTarget {
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tiago_bartender_msgs::MoveToTargetAction>
      as_mtt_;
  actionlib::SimpleActionServer<tiago_bartender_msgs::FindClosestTargetAction>
      as_fct_;
  std::string mtt_action_name_;
  std::string fct_action_name_;
  tiago_bartender_msgs::FindClosestTargetResult fct_res_;
  tiago_bartender_msgs::MoveToTargetResult mtt_res_;

public:
  MoveToTarget(std::string mtt_name, std::string fct_name)
      : as_mtt_(nh_, mtt_name, boost::bind(&MoveToTarget::executeMTT, this, _1),
                false),
        mtt_action_name_(mtt_name),
        as_fct_(nh_, fct_name, boost::bind(&MoveToTarget::executeFCT, this, _1),
                false),
        fct_action_name_(fct_name), ac_("move_base", true), psi_() {
    ros::NodeHandle bn("tiago_bartender");
    XmlRpc::XmlRpcValue lines;
    bn.getParam("move_to_target_lines", lines);
    ROS_ASSERT(lines.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::iterator i = lines.begin(); i != lines.end();
         i++) {
      ROS_ASSERT(i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      line_segment_a_x_.push_back(
          static_cast<double>(i->second["start_point_x"]));
      line_segment_a_y_.push_back(
          static_cast<double>(i->second["start_point_y"]));
      line_segment_b_x_.push_back(
          static_cast<double>(i->second["end_point_x"]));
      line_segment_b_y_.push_back(
          static_cast<double>(i->second["end_point_y"]));

      ori_x_.push_back(static_cast<double>(i->second["orientation_x"]));
      ori_y_.push_back(static_cast<double>(i->second["orientation_y"]));
      ori_z_.push_back(static_cast<double>(i->second["orientation_z"]));
      ori_w_.push_back(static_cast<double>(i->second["orientation_w"]));
    }

    default_frame_ = "map";

    if (line_segment_a_x_.size() != line_segment_a_y_.size() ||
        line_segment_a_y_.size() != line_segment_b_x_.size() ||
        line_segment_b_x_.size() != line_segment_b_y_.size() ||
        line_segment_b_y_.size() != ori_x_.size() ||
        ori_y_.size() != ori_z_.size() || ori_z_.size() != ori_w_.size()) {
      ROS_ERROR_STREAM(
          "All vectors for the line segments must be the same size.");
    }

    look_at_client_ = nh_.serviceClient<tiago_bartender_msgs::LookAt>(
        "head_controller/look_at_service");

    // wait for the action server to come up
    while (!ac_.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for move base action server to come up");
    }

    vis_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "move_to_target_marker", 20, true);

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = default_frame_;
      marker.id = 87264;
      marker.ns = "lines";
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
      marker.scale.y = 1;
      marker.scale.z = 1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      for (size_t i = 0; i < line_segment_a_x_.size(); i++) {
        double z = 0.1;
        marker.points.emplace_back();
        marker.points.back().x = line_segment_a_x_[i];
        marker.points.back().y = line_segment_a_y_[i];
        marker.points.back().z = z;
        marker.points.emplace_back();
        marker.points.back().x = line_segment_b_x_[i];
        marker.points.back().y = line_segment_b_y_[i];
        marker.points.back().z = z;
      }
      vis_pub_.publish(marker);
    }

    as_mtt_.start();
    as_fct_.start();
  }

  void executeMTT(const tiago_bartender_msgs::MoveToTargetGoalConstPtr &goal) {
    geometry_msgs::PoseStamped target_pose;
    if (!goal->target.empty())
      target_pose = get_pose_from_id(goal->target);
    else
      target_pose = goal->target_pose;

    if (target_pose.header.frame_id.empty()) {
      as_mtt_.setAborted();
      return;
    }

    // Point head to target
    if (goal->look_at_target) {
      tiago_bartender_msgs::LookAt srv;
      srv.request.target_point.header = target_pose.header;
      srv.request.target_point.point = target_pose.pose.position;
      if (!look_at_client_.call(srv)) {
        ROS_ERROR("Failed to call look_at_service");
      }
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
  }

  void
  executeFCT(const tiago_bartender_msgs::FindClosestTargetGoalConstPtr &goal) {
    geometry_msgs::PoseStamped target_pose;
    target_pose = get_pose_from_type(goal->target_type, goal->target_pose);

    if (target_pose.header.frame_id.empty()) {
      as_fct_.setAborted();
      return;
    }

    // Point head to target
    if (goal->look_at_target) {
      tiago_bartender_msgs::LookAt srv;
      srv.request.target_point.header = target_pose.header;
      srv.request.target_point.point = target_pose.pose.position;
      if (!look_at_client_.call(srv)) {
        ROS_ERROR("Failed to call look_at_service");
      }
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
  }

  inline static double squared(double v) { return v * v; }

  // set fct to true if the function is used by the find closest_target action
  // server
  void move_to_target_pose(geometry_msgs::PoseStamped target_pose, bool fct) {

    ROS_INFO("move to target frame id %s", target_pose.header.frame_id.c_str());
    ROS_INFO("move to target default frame %s", default_frame_.c_str());

    geometry_msgs::PoseStamped matched_pose;
    matched_pose.header.frame_id = default_frame_;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < line_segment_a_x_.size(); i++) {
      /*double Ax = line_segment_a_x_[i];
      double Ay = line_segment_a_y_[i];
      double Bx = line_segment_b_x_[i];
      double By = line_segment_b_y_[i];
      double Cx = target_pose.pose.position.x;
      double Cy = target_pose.pose.position.y;*/

      /*double t = ((Cx - Ax) * (Bx - Ax) + (Cy - Ay) * (By - Ay)) /
                 (std::pow(Bx - Ax, 2.0) + std::pow(By - Ay, 2.0));
      double Dx = Ax + t * (Bx - Ax);
      double Dy = Ay + t * (By - Ay);*/

      Eigen::Vector2d line_point_a =
          Eigen::Vector2d(line_segment_a_x_[i], line_segment_a_y_[i]);
      Eigen::Vector2d line_point_b =
          Eigen::Vector2d(line_segment_b_x_[i], line_segment_b_y_[i]);
      Eigen::Vector2d target_point = Eigen::Vector2d(
          target_pose.pose.position.x, target_pose.pose.position.y);

      double line_length = (line_point_b - line_point_a).norm();

      Eigen::Vector2d line_direction =
          (line_point_b - line_point_a).normalized();

      double fpos = target_point.dot(line_direction);
      if (fpos < 0)
        fpos = 0;
      if (fpos > line_length)
        fpos = line_length;

      Eigen::Vector2d closest_point = line_point_a + line_direction * fpos;

      // double distance = std::abs(target_pose.pose.position.x - Dx) +
      // std::abs(target_pose.pose.position.y - Dy);
      double distance = (closest_point - target_point).norm();

      if (distance < min_distance) {
        min_distance = distance;
        matched_pose.pose.position.x = closest_point.x();
        matched_pose.pose.position.y = closest_point.y();
        matched_pose.pose.position.z = 0.0;
        matched_pose.pose.orientation.x = ori_x_[i];
        matched_pose.pose.orientation.y = ori_y_[i];
        matched_pose.pose.orientation.z = ori_z_[i];
        matched_pose.pose.orientation.w = ori_w_[i];
      }
    }

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = default_frame_;
      marker.id = 412;
      marker.ns = "target";
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = matched_pose.pose;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      vis_pub_.publish(marker);
    }

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = default_frame_;
      marker.id = 1523;
      marker.ns = "closest_point";
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = matched_pose.pose;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      vis_pub_.publish(marker);
    }

    move_base_msgs::MoveBaseGoal target;

    target.target_pose = matched_pose;

    ac_.sendGoal(target);

    while (!ac_.waitForResult(ros::Duration(0.5)) && ros::ok()) {
      if (as_mtt_.isPreemptRequested() && !fct) {
        ROS_INFO("Moving to target was aborted.");
        ac_.cancelGoal();
        as_mtt_.setPreempted();
        return;
      }

      if (as_fct_.isPreemptRequested() && fct) {
        ROS_INFO("Finding cloest target was aborted.");
        ac_.cancelGoal();
        as_fct_.setPreempted();
        return;
      }
    }

    if (!fct) {
      mtt_res_.pose_result = matched_pose;
      mtt_res_.target_pose_result = target_pose;

      if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        as_mtt_.setSucceeded(mtt_res_);
      else
        as_mtt_.setAborted(mtt_res_);
    }

    if (fct) {
      fct_res_.pose_result = matched_pose;
      fct_res_.target_pose_result = target_pose;

      if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        as_fct_.setSucceeded(fct_res_);
      else
        as_fct_.setAborted(fct_res_);
    }
  }

  geometry_msgs::PoseStamped get_pose_from_id(std::string target_id) {
    geometry_msgs::PoseStamped pose;
    std::map<std::string, moveit_msgs::CollisionObject> objects =
        psi_.getObjects(std::vector<std::string>({target_id}));
    if (objects.empty()) {
      ROS_ERROR("Target object not in planning scene");
      return pose;
    }
    moveit_msgs::CollisionObject object = objects[target_id];
    pose.header = object.header;

    if (!object.primitive_poses.empty()) {
      pose.pose = object.primitive_poses.at(0);
    } else if (!object.mesh_poses.empty()) {
      pose.pose = object.mesh_poses.at(0);
    }
    tf_listener_.transformPose(default_frame_, pose, pose);
    return pose;
  }

  geometry_msgs::PoseStamped
  get_pose_from_type(std::string type, geometry_msgs::PoseStamped pose) {
    geometry_msgs::PoseStamped matched_pose;
    std::map<std::string, moveit_msgs::CollisionObject> objects =
        psi_.getObjects();
    tf_listener_.transformPose(default_frame_, pose, pose);

    double min_distance = std::numeric_limits<double>::max();
    for (auto object : objects) {
      moveit_msgs::CollisionObject co = object.second;
      if (co.id.find(type) == std::string::npos) {
        continue;
      }
      geometry_msgs::PoseStamped target_pose;
      target_pose.header = co.header;

      if (!co.primitive_poses.empty()) {
        target_pose.pose = co.primitive_poses.at(0);
      } else if (!co.mesh_poses.empty()) {
        target_pose.pose = co.mesh_poses.at(0);
      }
      tf_listener_.transformPose(default_frame_, target_pose, target_pose);

      double distance =
          std::abs(target_pose.pose.position.x - pose.pose.position.x) +
          std::abs(target_pose.pose.position.y - pose.pose.position.y);
      if (distance < min_distance) {
        min_distance = distance;
        matched_pose = target_pose;
        fct_res_.target_id = co.id;
      }
    }

    return matched_pose;
  }

private:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  ros::ServiceClient look_at_client_;

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_to_target_node");
  ros::NodeHandle nh;
  MoveToTarget mtb("move_to_target", "find_closest_target");
  ros::spin();
}
