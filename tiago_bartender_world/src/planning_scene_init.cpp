#include <actionlib/server/simple_action_server.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

class PlanningSceneInit {
public:
  PlanningSceneInit() : psi_() {
    ros::NodeHandle bn("tiago_bartender");
    XmlRpc::XmlRpcValue objects;
    bn.getParam("scene_objects", objects);

    ROS_ASSERT(objects.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (XmlRpc::XmlRpcValue::iterator i = objects.begin(); i != objects.end();
         i++) {
      ROS_ASSERT(i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      object_ids_.push_back(static_cast<std::string>(i->first));
      object_types_.push_back(static_cast<std::string>(i->second["type"]));
      frames_.push_back(static_cast<std::string>(i->second["frame"]));
      pos_x_.push_back(static_cast<double>(i->second["pos_x"]));
      pos_y_.push_back(static_cast<double>(i->second["pos_y"]));
      pos_z_.push_back(static_cast<double>(i->second["pos_z"]));
    }
  }

  void start_service() {
    init_server_ = nh_.advertiseService("planning_scene/init_planning_scene",
                                        &PlanningSceneInit::init_service, this);
  }

  bool init_scene() {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<moveit_msgs::ObjectColor> object_colors;

    for (size_t i = 0; i < object_ids_.size(); ++i) {
      moveit_msgs::CollisionObject collision_object;
      moveit_msgs::ObjectColor object_color;
      collision_object.header.frame_id = frames_[i];
      collision_object.id = object_ids_[i];
      object_color.id = object_ids_[i];
      ROS_INFO_STREAM("add " << collision_object.id << " of type "
                             << object_types_[i] << " to planning scene.");

      if (object_types_[i] == "bottle") {
        shapes::Mesh *m = shapes::createMeshFromResource(
            "package://tams_bartender_recognition/meshes/bottle_small.stl");
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m, mesh_msg);
        shape_msgs::Mesh mesh;
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        collision_object.meshes.push_back(mesh);

        object_color.color.r = 1.0;
        object_color.color.a = 1.0;

        geometry_msgs::Pose pose;
        pose.position.x = pos_x_[i];
        pose.position.y = pos_y_[i];
        pose.position.z = pos_z_[i];
        pose.orientation.w = 1.0;
        collision_object.mesh_poses.push_back(pose);
        collision_objects.push_back(collision_object);

        object_colors.push_back(object_color);
      } else if (object_types_[i] == "glass") {
        shapes::Mesh *m = shapes::createMeshFromResource(
            "package://tams_bartender_recognition/meshes/glass-binary.stl");
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m, mesh_msg);
        shape_msgs::Mesh mesh;
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        collision_object.meshes.push_back(mesh);

        object_color.color.r = 1.0;
        object_color.color.a = 1.0;
        object_color.color.g = 1.0;

        geometry_msgs::Pose pose;
        pose.position.x = pos_x_[i];
        pose.position.y = pos_y_[i];
        pose.position.z = pos_z_[i];
        pose.orientation.w = 1.0;
        collision_object.mesh_poses.push_back(pose);
        collision_objects.push_back(collision_object);

        object_colors.push_back(object_color);
      } else if (object_types_[i] == "small_table") {
        Eigen::Vector3d table_scale;
        table_scale[0] = 1.0;
        table_scale[1] = 0.8;
        table_scale[2] = 1.0;

        shapes::Mesh *m = shapes::createMeshFromResource(
            "package://tiago_bartender_world/meshes/paltable.stl", table_scale);
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m, mesh_msg);
        shape_msgs::Mesh mesh;
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        collision_object.meshes.push_back(mesh);

        object_color.color.r = 222.0 / 255.0;
        object_color.color.g = 184.0 / 255.0;
        object_color.color.b = 135.0 / 255.0;
        object_color.color.a = 1.0;

        geometry_msgs::Pose pose;
        pose.position.x = pos_x_[i];
        pose.position.y = pos_y_[i];
        pose.position.z = pos_z_[i];
        pose.orientation.w = 1.0;
        collision_object.mesh_poses.push_back(pose);
        collision_objects.push_back(collision_object);

        object_colors.push_back(object_color);
      } else if (object_types_[i] == "large_table") {
        Eigen::Vector3d table_scale;
        table_scale[0] = 1.0;
        table_scale[1] = 1.0;
        table_scale[2] = 1.0;

        shapes::Mesh *m = shapes::createMeshFromResource(
            "package://tiago_bartender_world/meshes/paltable.stl", table_scale);
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m, mesh_msg);
        shape_msgs::Mesh mesh;
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        collision_object.meshes.push_back(mesh);

        object_color.color.r = 222.0 / 255.0;
        object_color.color.g = 184.0 / 255.0;
        object_color.color.b = 135.0 / 255.0;
        object_color.color.a = 1.0;

        geometry_msgs::Pose pose;
        pose.position.x = pos_x_[i];
        pose.position.y = pos_y_[i];
        pose.position.z = pos_z_[i];
        pose.orientation.w = 1.0;
        collision_object.mesh_poses.push_back(pose);
        collision_objects.push_back(collision_object);

        object_colors.push_back(object_color);
      }
    }

    return psi_.applyCollisionObjects(collision_objects, object_colors);
  }

private:
  bool init_service(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
    init_scene();
    return true;
  }

  moveit::planning_interface::PlanningSceneInterface psi_;
  std::vector<std::string> object_ids_;
  std::vector<std::string> object_types_;
  std::vector<double> pos_x_;
  std::vector<double> pos_y_;
  std::vector<double> pos_z_;
  std::vector<std::string> frames_;

  ros::ServiceServer init_server_;

  ros::NodeHandle nh_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_scene_init");
  PlanningSceneInit psinit;

  while (!psinit.init_scene())
    ros::Duration(5.0).sleep();

  psinit.start_service();
  ros::spin();
}
