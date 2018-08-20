#include <ros/ros.h>
#include <functional>
#include <std_msgs/String.h>
#include <tiago_bartender_navigation/MoveToTargetAction.h>
#include <tiago_bartender_navigation/FindClosestTargetAction.h>
//#include <tiago_bartender_mtc/PickBottleAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tiago_bartender_speech/BartenderSpeechAction.h>
#include <tiago_bartender_menu/TakeOrderAction.h>
#include <tiago_bartender_behavior/LookAt.h>

class StateMachine
{
public:
  StateMachine() : mtt_client_("move_to_target", true),
                   fct_client_("find_closest_target", true),
                   fjt_client_("torso_controller/follow_joint_trajectory", true),
                   bs_client_("bartender_speech_action", true),
                   to_client_("menu/take_order", true)
  {
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("bartender_state_marker", 0);
    look_at_client_ = nh_.serviceClient<tiago_bartender_behavior::LookAt>("head_controller/look_at_service");

    // prepare the marker for use later on
    marker_.header.frame_id = "base_footprint";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = 0.25;
    marker_.pose.position.y = 0.25;
    marker_.pose.position.z = 2.0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.25;
    marker_.scale.y = 0.25;
    marker_.scale.z = 0.25;
    marker_.color.a = 1.0;
    marker_.color.r = 1.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.0;

    // without this sleep the first marker is not published
    ros::Duration(1.0).sleep();

    while(!mtt_client_.waitForServer(ros::Duration(1.0)) || !fct_client_.waitForServer(ros::Duration(1.0)) || !fjt_client_.waitForServer(ros::Duration(1.0)) || !bs_client_.waitForServer(ros::Duration(1.0)) || !to_client_.waitForServer(ros::Duration(1.0)))
    {
      ROS_ERROR_STREAM("tiago bartender state machine waits for all action servers to start.");
    }
  }

  void run()
  {
    while(true)
    {
      state(this);
    }
  }


private:
  void state_init()
  {
    ROS_INFO("idling");
    extend_torso();

    publish_marker("state_init");
    ros::NodeHandle node;

    // start idle behavior
    ros::ServiceClient client = node.serviceClient<std_srvs::SetBool>("switch_simple_idle");
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (!client.call(srv))
    {
      ROS_ERROR("Failed to call service switch_simple_idle");
      state = &StateMachine::state_init;
      return;
    }

    // Wait for a person to be detected
    bool received = false;
    geometry_msgs::PointStamped person_position;
    auto selection = node.subscribe<geometry_msgs::PointStamped>("/person_detection", 1, boost::function<void(geometry_msgs::PointStamped)>(
      [&](geometry_msgs::PointStamped p) {
        ROS_INFO("person pose");
        received = true;
        person_position = p;
      }
    ));
    while(true)
    {
      ros::Duration(0.1).sleep();
      if(received) break;
    }

    // stop idle behavior
    srv.request.data = false;
    if (!client.call(srv))
    {
      ROS_ERROR("Failed to call service switch_simple_idle");
      state = &StateMachine::state_init;
      return;
    }
    sleep(0.1);

    state = [person_position](StateMachine* m) { m->state_move_to_person(person_position); };
  }

  void state_move_to_person(const geometry_msgs::PointStamped person_position)
  {
    ROS_INFO("Moving to person");
    publish_marker("state_take_move_to_person");
    tiago_bartender_navigation::MoveToTargetGoal mtt_goal;
    geometry_msgs::PoseStamped target_pose;
    target_pose.header = person_position.header;
    target_pose.pose.position = person_position.point;

    move_to_pose(target_pose, true);
    state = [person_position](StateMachine* m) { m->state_ask_order(person_position, 0); };
  }

  void state_ask_order(const geometry_msgs::PointStamped person_position, size_t iteration)
  {
    ROS_INFO("Ask for order");
    publish_marker("state_ask_order");
    voice_command("ask_order");
    state = [person_position, iteration](StateMachine* m) { m->state_take_order(person_position, iteration); };
  }

  void state_menu_not_found(const geometry_msgs::PointStamped person_position, size_t iteration)
  {
    ROS_INFO("Ask customer to get the menu in the correct position.");
    publish_marker("state_menu_not_found");
    voice_command("menu_not_found");
    state = [person_position, iteration](StateMachine* m) { m->state_take_order(person_position, iteration); };
  }
  
  void state_abort_order()
  {
    ROS_INFO_STREAM("Abort ordering process.");
    publish_marker("state_abort_order");
    voice_command("abort_order");
    state = &StateMachine::state_init;
  }

  void state_take_order(const geometry_msgs::PointStamped person_position, size_t iteration)
  {
    ROS_INFO("Taking order");
    ++iteration;
    publish_marker("state_take_order");
    // Point head down to look at menu
    tiago_bartender_behavior::LookAt srv;
    srv.request.direction = "down";
    if (!look_at_client_.call(srv))
    {
      ROS_ERROR("Failed to call look_at_service");
    }
    ros::Duration(2.0).sleep();

    // take order action
    tiago_bartender_menu::TakeOrderGoal goal;
    goal.timeout = ros::Duration(30.0);
    to_client_.sendGoal(goal);
    while(!to_client_.waitForResult(ros::Duration(20.0)))
      ROS_INFO("Waiting for take order action result.");

    auto result = to_client_.getResult();
    ROS_INFO_STREAM("Result of order action, status: " << result->status << " selection: " << result->selection);

    // Point head to look at the customer
    srv.request.direction = "";
    srv.request.target_point.header = person_position.header;
    srv.request.target_point.point = person_position.point;
    if (!look_at_client_.call(srv))
    {
      ROS_ERROR("Failed to call look_at_service");
    }
    ros::Duration(2.0).sleep();

    // process result and choose next state accordingly
    if((result->status == "timeout" || result->status == "no_menu_card_detected") && iteration >= 3)
    {
      state = &StateMachine::state_abort_order;
      return;
    }
    else if(result->status == "timeout")
    {
      state = [person_position, iteration](StateMachine* m) { m->state_ask_order(person_position, iteration); };
      return;
    }
    else if(result->status == "no_menu_card_detected")
    {
      state = [person_position, iteration](StateMachine* m) { m->state_menu_not_found(person_position, iteration); };
      return;
    }

    // successful order
    std::string bottle_name = result->selection;

    last_person_position_ = person_position;

    state = [bottle_name](StateMachine* m) { m->state_move_to_bottle(bottle_name); };
  }

  void state_move_to_bottle(const std::string& bottle_name)
  {
    publish_marker("state_move_to_bottle");
    bool success = move_to_target(bottle_name, true);
    if(success)
      state = [bottle_name](StateMachine* m) { m->state_update_scene(bottle_name); };
    else
      state = [bottle_name](StateMachine* m) { m->state_move_to_bottle(bottle_name); };
  }

  void state_update_scene(const std::string& bottle_name)
  {
    // todo: replace this place holder with actual update method
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<std_srvs::Empty>("dummy_planning_scene/update_planning_scene");
    std_srvs::Empty srv;
    if (!client.call(srv))
    {
      ROS_ERROR("Failed to call service update_planning_scene");
      state = [bottle_name](StateMachine* m) { m->state_update_scene(bottle_name); };
      return;
    }
    state = [bottle_name](StateMachine* m) { m->state_grasp_bottle(bottle_name); };
  }
 
  void state_grasp_bottle(const std::string& bottle_name)
  {
    ROS_INFO("grasping");
    publish_marker("state_grasp_bottle");

    /*
    actionlib::SimpleActionClient<tiago_bartender_mtc::PickBottleAction> client("tiago_bottle_pick", true);
    client.waitForServer();
    tiago_bartender_mtc::PickBottleGoal goal;
    goal.bottle_id = bottle_name;
    client.sendGoal(goal);
    while(!client.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for bottle grasp result.");
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully grasped to bottle.");
    else
      ROS_ERROR("Grasping bottle aborted.");
    */

    state = &StateMachine::state_move_to_glass;
  }
  
  void state_move_to_glass()
  {
    ROS_INFO("moving to glass");
    publish_marker("state_move_to_glass");
    geometry_msgs::PoseStamped person_pose;
    person_pose.header = last_person_position_.header;
    person_pose.pose.position = last_person_position_.point;
    std::string target_id_result;
    bool success = find_closest_target("glass", person_pose, true, target_id_result);
    if(success)
      state = [target_id_result](StateMachine* m) { m->state_pour(target_id_result); };
    else
      state = &StateMachine::state_move_to_glass;
  }

  void state_pour(const std::string& glass_id)
  {
    ROS_INFO("pouring");
    publish_marker("state_pour");
    ros::Duration(1).sleep();
    state = &StateMachine::state_move_back_to_shelf;
  }

  void state_move_back_to_shelf()
  {
    ROS_INFO("moving back to shelf");
    publish_marker("state_move_back_to_shelf");
    bool success = move_to_pose(last_object_target_pose_, true);
    if(success)
      state = &StateMachine::state_put_back_bottle;
    else
      state = &StateMachine::state_move_back_to_shelf;
  }

  void state_put_back_bottle()
  {
    ROS_INFO("putting bottle back");
    publish_marker("state_put_back_bottle");
    ros::Duration(1).sleep();
    state = &StateMachine::state_init;
  }
 
  bool move_to_target(const std::string& target_name, bool look_at_target)
  {
    ROS_INFO("moving to target %s", target_name.c_str());
    tiago_bartender_navigation::MoveToTargetGoal goal;
    goal.target = target_name;
    goal.look_at_target = look_at_target;
    mtt_client_.sendGoal(goal);
    while(!mtt_client_.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for move_to_target result.");
    if(mtt_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Successfully moved to target.");
      last_object_target_pose_ = mtt_client_.getResult()->target_pose_result;
      return true;
    }
    else
    {
      ROS_ERROR("Moving to target aborted.");
      return false;
    }
  }

  bool find_closest_target(const std::string& type_name, const geometry_msgs::PoseStamped& pose, const bool look_at_target, std::string& target_id_result)
  {
    ROS_INFO("Finding closest target of type %s", type_name.c_str());
    tiago_bartender_navigation::FindClosestTargetGoal goal;
    goal.target_type = type_name;
    goal.target_pose = pose;
    goal.look_at_target = look_at_target;
    fct_client_.sendGoal(goal);
    while(!fct_client_.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for find_closest_target result.");
    if(fct_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Successfully found and moved to target.");
      target_id_result = fct_client_.getResult()->target_id;
      return true;
    }
    else
    {
      ROS_ERROR("Find closest target aborted.");
      return false;
    }
  }

  bool move_to_pose(const geometry_msgs::PoseStamped pose, bool look_at_target)
  {
    ROS_INFO("moving to pose.");
    tiago_bartender_navigation::MoveToTargetGoal goal;
    goal.target_pose = pose;
    goal.look_at_target = look_at_target;
    mtt_client_.sendGoal(goal);
    while(!mtt_client_.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for move_to_target result.");
    if(mtt_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Successfully moved to target.");
      return true;
    }
    else
    {
      ROS_ERROR("Moving to target aborted.");
      return false;
    }
  }

  void publish_marker(const std::string& text)
  {
    marker_.text = text;
    marker_pub_.publish(marker_);
  }

  void extend_torso()
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory torso_command;
    torso_command.joint_names.push_back("torso_lift_joint");
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions.push_back(0.35);
    jtp.time_from_start = ros::Duration(0.5);
    torso_command.points.push_back(jtp);
    goal.trajectory = torso_command;

    fjt_client_.sendGoal(goal);
    while(!fjt_client_.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting to extend torso.");
    if(fjt_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully extended torso.");
  }

  void voice_command(std::string audio_id)
  {
    tiago_bartender_speech::BartenderSpeechGoal speech_goal;
    speech_goal.id = audio_id;
    bs_client_.sendGoal(speech_goal);
    while(!bs_client_.waitForResult(ros::Duration(5.0)))
      ROS_INFO("Waiting for bartender speech action result.");
    if(bs_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_ERROR_STREAM("bartender speech action failed.");
  }
  
  std::function<void(StateMachine*)> state = &StateMachine::state_init;

  actionlib::SimpleActionClient<tiago_bartender_navigation::MoveToTargetAction> mtt_client_;
  actionlib::SimpleActionClient<tiago_bartender_navigation::FindClosestTargetAction> fct_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> fjt_client_;
  actionlib::SimpleActionClient<tiago_bartender_speech::BartenderSpeechAction> bs_client_;
  actionlib::SimpleActionClient<tiago_bartender_menu::TakeOrderAction> to_client_;

  ros::ServiceClient look_at_client_;

  geometry_msgs::PointStamped last_person_position_;
  // pose of the object the robot moved to most recently
  geometry_msgs::PoseStamped last_object_target_pose_;
  ros::Publisher marker_pub_;
  visualization_msgs::Marker marker_;
  ros::NodeHandle nh_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  StateMachine sm;
  sm.run();
}
