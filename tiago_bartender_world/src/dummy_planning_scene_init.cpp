#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gazebo_msgs/LinkStates.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <mutex>
#include <gazebo_msgs/GetLinkState.h>

class PlanningSceneInit
{
public:
  PlanningSceneInit() : psi_(), objects_found_(false)
  {
    ros::NodeHandle nh;
    obj_pose_sub_ = nh.subscribe("/gazebo/link_states", 1000, &PlanningSceneInit::obj_pose_callback, this);

    init_server_ = nh.advertiseService("dummy_planning_scene/init_planning_scene", &PlanningSceneInit::init_service, this);
    update_server_ = nh.advertiseService("dummy_planning_scene/update_planning_scene", &PlanningSceneInit::update_service, this);
    link_state_client_ = nh.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");

    gazebo_planning_scene_mapping_ = {
      {"bottle_1::bottle_2_coke", "bottle_1"},
      { "bottle_2::bottle_2_fanta", "bottle_2"}, 
      {"bottle_3::bottle_2_pepsi", "bottle_3"},
      {"bottle_4::bottle_coke", "bottle_4"},
      {"bottle_5::bottle_fanta", "bottle_5"},
      { "bottle_6::bottle_pepsi", "bottle_6"},
      {"glass_1::yellow_glass", "glass_1"},
      {"glass_2::yellow_glass", "glass_2"},
      {"glass_3::yellow_glass", "glass_3"},
      {"bar_model::counter", "counter"},
      {"bar_model::table1", "table1"},
      {"bar_model::table2", "table2"},
      {"bar_model::table3", "table3"}};
  }

  void update_scene()
  {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_footprint";
    geometry_msgs::Pose pose;

    for(std::pair<std::string, std::string> object : gazebo_planning_scene_mapping_)
    {
      gazebo_msgs::GetLinkState srv;
      srv.request.link_name = object.first;
      srv.request.reference_frame = "tiago_custom::base_footprint";
      if (!link_state_client_.call(srv) || !srv.response.success)
      {
        ROS_ERROR("Failed to call service get_link_state with %s", object.first.c_str());
        return;
      }
      if(object.first.find("bottle") != std::string::npos)
        srv.response.link_state.pose.position.z += 0.005;

      collision_object.mesh_poses.resize(1);
      collision_object.mesh_poses[0] = srv.response.link_state.pose;
      collision_object.id = object.second;
      collision_object.operation = moveit_msgs::CollisionObject::MOVE;
      collision_objects.push_back(collision_object);
    }
    psi_.applyCollisionObjects(collision_objects);
  }

  void init_scene()
  {
    //std::lock_guard<std::mutex> guard(pose_mutex_);

    while(!objects_found_ && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<moveit_msgs::ObjectColor> object_colors;
    moveit_msgs::CollisionObject collision_object;
    moveit_msgs::ObjectColor object_color;

    collision_object.header.frame_id = "world";
    shapes::Mesh* m = shapes::createMeshFromResource("package://tiago_bartender_world/meshes/bottle_2_coke.dae");
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::Mesh mesh;
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    collision_object.meshes.push_back(mesh);

    object_color.color.r = 1.0;
    object_color.color.a = 1.0;

    for(auto pose : bottle_poses_)
    {
      collision_object.id = pose.first;
      collision_object.mesh_poses.push_back(pose.second);
      collision_objects.push_back(collision_object);
      collision_object.mesh_poses.clear();

      object_color.id = pose.first;
      object_colors.push_back(object_color);
    }
    Eigen::Vector3d glass_scale;
    glass_scale[0] = 0.001;
    glass_scale[1] = 0.001;
    glass_scale[2] = 0.001;
    m = shapes::createMeshFromResource("package://tiago_bartender_world/meshes/ikea_glass_binary.stl", glass_scale);
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    collision_object.meshes[0] = mesh;

    object_color.color.g = 1.0;

    for(auto pose : glass_poses_)
    {
      collision_object.id = pose.first;
      collision_object.mesh_poses.push_back(pose.second);
      collision_objects.push_back(collision_object);
      collision_object.mesh_poses.clear();

      object_color.id = pose.first;
      object_colors.push_back(object_color);
    }

    // building the bar scene
    Eigen::Vector3d bar_scale;
    bar_scale[0] = 0.00075;
    bar_scale[1] = 0.00075;
    bar_scale[2] = 0.00075;

    Eigen::Vector3d table_scale;
    table_scale[0] = 0.001;
    table_scale[1] = 0.001;
    table_scale[2] = 0.001;


    object_color.color.r = 222.0/255.0;
    object_color.color.g = 184.0/255.0;
    object_color.color.b = 135.0/255.0;

    // Use solid primitives for the legs of the table
    shape_msgs::SolidPrimitive prim_msg;
    prim_msg.type = shape_msgs::SolidPrimitive::BOX;
    prim_msg.dimensions = {0.02, 0.02, 0.73};
    geometry_msgs::Pose prim_pose;

    for(auto pose : bar_poses_)
    {
      if(pose.first == "counter")
      {
        m = shapes::createMeshFromResource("package://tiago_bartender_world/meshes/bartresen_binary.stl", bar_scale);
      }
      else if(pose.first == "table1" || pose.first == "table2")
      {
        m = shapes::createMeshFromResource("package://tiago_bartender_world/meshes/table_plate_160x80x2.dae", table_scale);

        collision_object.primitives = {prim_msg, prim_msg, prim_msg, prim_msg};

        prim_pose = pose.second;
        prim_pose.position.y += 1.6/2.0-0.02/2.0;
        prim_pose.position.x += 0.8/2.0-0.02/2.0;
        prim_pose.position.z += 0.73/2.0;
        collision_object.primitive_poses.push_back(prim_pose);

        prim_pose = pose.second;
        prim_pose.position.y += -1.6/2.0+0.02/2.0;
        prim_pose.position.x += 0.8/2.0-0.02/2.0;
        prim_pose.position.z += 0.73/2.0;
        collision_object.primitive_poses.push_back(prim_pose);

        prim_pose = pose.second;
        prim_pose.position.y += 1.6/2.0-0.02/2.0;
        prim_pose.position.x += -0.8/2.0+0.02/2.0;
        prim_pose.position.z += 0.73/2.0;
        collision_object.primitive_poses.push_back(prim_pose);

        prim_pose = pose.second;
        prim_pose.position.y += -1.6/2.0+0.02/2.0;
        prim_pose.position.x += -0.8/2.0+0.02/2.0;
        prim_pose.position.z += 0.73/2.0;
        collision_object.primitive_poses.push_back(prim_pose);

        pose.second.position.z += 0.74;

      }
      else if(pose.first == "table3")
      {
        m = shapes::createMeshFromResource("package://tiago_bartender_world/meshes/table_plate_140x70x2.dae", table_scale);

        collision_object.primitives = {prim_msg, prim_msg, prim_msg, prim_msg};

        prim_pose = pose.second;
        prim_pose.position.x += 1.4/2.0-0.02/2.0;
        prim_pose.position.y += 0.7/2.0-0.02/2.0;
        prim_pose.position.z += 0.73/2.0;
        collision_object.primitive_poses.push_back(prim_pose);

        prim_pose = pose.second;
        prim_pose.position.x += -1.4/2.0+0.02/2.0;
        prim_pose.position.y += 0.7/2.0-0.02/2.0;
        prim_pose.position.z += 0.73/2.0;
        collision_object.primitive_poses.push_back(prim_pose);

        prim_pose = pose.second;
        prim_pose.position.x += 1.4/2.0-0.02/2.0;
        prim_pose.position.y += -0.7/2.0+0.02/2.0;
        prim_pose.position.z += 0.73/2.0;
        collision_object.primitive_poses.push_back(prim_pose);

        prim_pose = pose.second;
        prim_pose.position.x += -1.4/2.0+0.02/2.0;
        prim_pose.position.y += -0.7/2.0+0.02/2.0;
        prim_pose.position.z += 0.73/2.0;
        collision_object.primitive_poses.push_back(prim_pose);

        pose.second.position.z += 0.74;
      }
      else
      {
        continue;
      }
      shapes::constructMsgFromShape(m, mesh_msg);
      mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
      collision_object.meshes[0] = mesh;

      collision_object.id = pose.first;
      collision_object.mesh_poses.push_back(pose.second);
      collision_objects.push_back(collision_object);
      collision_object.mesh_poses.clear();
      collision_object.primitive_poses.clear();
      collision_object.primitives.clear();

      object_color.id = pose.first;
      object_colors.push_back(object_color);
    }
    psi_.applyCollisionObjects(collision_objects, object_colors);
  }
private:
  void obj_pose_callback(const gazebo_msgs::LinkStates::ConstPtr& msg)
  {
    //std::lock_guard<std::mutex> guard(pose_mutex_);

    bottle_poses_.clear();
    glass_poses_.clear();
    bar_poses_.clear();
    for(size_t i = 0; i < msg->name.size(); i++)
    {
      if(msg->name[i].find("bottle") != std::string::npos)
        bottle_poses_[msg->name[i].substr(0, msg->name[i].find(":"))] = msg->pose[i];
      else if(msg->name[i].find("glass") != std::string::npos)
        glass_poses_[msg->name[i].substr(0, msg->name[i].find(":"))] = msg->pose[i];
      else if(msg->name[i].find("bar_model") != std::string::npos)
        bar_poses_[msg->name[i].substr(msg->name[i].find(":") + 2)] = msg->pose[i];
    }
    if(bottle_poses_.size() > 0 || glass_poses_.size() > 0 || bar_poses_.size() > 0)
      objects_found_ = true;
  }

  bool init_service(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    init_scene();
    //update_scene();
    return true;
  }

  bool update_service(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    update_scene();
    return true;
  }

  moveit::planning_interface::PlanningSceneInterface psi_;
  ros::Subscriber obj_pose_sub_;
  std::map<std::string, geometry_msgs::Pose> bottle_poses_;
  std::map<std::string, geometry_msgs::Pose> glass_poses_;
  std::map<std::string, geometry_msgs::Pose> bar_poses_;
  std::mutex pose_mutex_;
  bool objects_found_;

  ros::ServiceServer init_server_;
  ros::ServiceServer update_server_;
  ros::ServiceClient link_state_client_;
  std::vector<std::pair<std::string, std::string>> gazebo_planning_scene_mapping_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_planning_scene_init");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  PlanningSceneInit psinit;
  psinit.init_scene();
  while(ros::ok())
  {

  }
}
