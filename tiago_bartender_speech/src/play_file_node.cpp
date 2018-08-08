#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <tiago_bartender_speech/BartenderSpeechAction.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <sound_play/SoundRequestAction.h>

#include  <random>
#include  <iterator>

template<typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator& g) {
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(g));
    return start;
}

template<typename Iter>
Iter select_randomly(Iter start, Iter end) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return select_randomly(start, end, gen);
}

class TiagoSpeech
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<tiago_bartender_speech::BartenderSpeechAction> as_;
  std::string action_name_;
public:
  TiagoSpeech(std::string action_name) : as_(nh_, action_name, boost::bind(&TiagoSpeech::executeCB, this, _1), false),
                                         action_name_(action_name),
                                         ac_("sound_play", true)
  {
    file_base_path_ = ros::package::getPath("tiago_bartender_speech");
    file_base_path_ = file_base_path_ + "/audio_files/";
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("sound_marker",0);

    // filling the audio_file_map
    audio_file_map_["ask_order"].push_back("ask_order.wav");
    ac_.waitForServer();
    as_.start();
  }
private:
  void executeCB(const tiago_bartender_speech::BartenderSpeechGoalConstPtr& goal)
  {
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
    marker.text = goal->id;
    vis_pub_.publish(marker);

    std::vector<std::string> files = audio_file_map_[goal->id];
    if(files.empty())
    {
      ROS_ERROR_STREAM("Id provided to bartender_speech_action is not known: " << goal->id);
      return;
    }
    std::string file_name = *select_randomly(files.begin(), files.end());
    sound_play::SoundRequestGoal sound_goal;
    sound_goal.sound_request.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_goal.sound_request.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_goal.sound_request.volume = 1.0;
    sound_goal.sound_request.arg = file_base_path_ + file_name;
    ac_.sendGoal(sound_goal);
    ac_.waitForResult();
    as_.setSucceeded();

    marker.action = visualization_msgs::Marker::DELETE;
    vis_pub_.publish(marker);
  }

  ros::Publisher vis_pub_;
  std::string file_base_path_;

  // maps from an identifier to a vector of filenames
  std::map<std::string, std::vector<std::string>> audio_file_map_;

  sound_play::SoundClient sound_client_;
  actionlib::SimpleActionClient<sound_play::SoundRequestAction> ac_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "play_file_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  TiagoSpeech tiago_speech("bartender_speech_action");
  while(ros::ok())
  {}
}
