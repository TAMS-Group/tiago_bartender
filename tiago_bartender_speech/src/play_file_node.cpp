#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <tiago_bartender_msgs/BartenderSpeechAction.h>
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
  actionlib::SimpleActionServer<tiago_bartender_msgs::BartenderSpeechAction> as_;
  std::string action_name_;
public:
  TiagoSpeech(std::string action_name) : as_(nh_, action_name, boost::bind(&TiagoSpeech::executeCB, this, _1), false),
                                         action_name_(action_name),
                                         ac_("sound_play", true)
  {
    ros::NodeHandle pn("~");
    pn.param("audio_files_base_path", file_base_path_, std::string(""));
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("sound_marker",0);

    // filling the audio_file_map
    //audio_file_map_["ask_order"].push_back("ask_order.wav");
    XmlRpc::XmlRpcValue audio_map;
    pn.getParam("audio_map", audio_map);
    ROS_ASSERT(audio_map.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for(XmlRpc::XmlRpcValue::iterator i = audio_map.begin(); i != audio_map.end(); i++)
    {
      ROS_ASSERT(i->second.getType()==XmlRpc::XmlRpcValue::TypeArray);
      std::string audio_id = static_cast<std::string>(i->first);
      audio_file_map_[audio_id];
      for(size_t j=0; j<i->second.size(); ++j)
      {
        std::string file_name = static_cast<std::string>(i->second[j]);
        audio_file_map_[audio_id].push_back(file_name);
      }
    }

    // prepare marker
    marker_.header.frame_id = "base_footprint";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = 0.0;
    marker_.pose.position.y = 0.0;
    marker_.pose.position.z = 1.75;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.25;
    marker_.scale.y = 0.25;
    marker_.scale.z = 0.25;
    marker_.color.a = 1.0;
    marker_.color.r = 1.0;
    marker_.color.g = 1.0;
    marker_.color.b = 1.0;

    ac_.waitForServer();
    as_.start();
  }
private:
  void executeCB(const tiago_bartender_msgs::BartenderSpeechGoalConstPtr& goal)
  {
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.text = goal->id;
    vis_pub_.publish(marker_);

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

    marker_.action = visualization_msgs::Marker::DELETE;
    vis_pub_.publish(marker_);
  }

  ros::Publisher vis_pub_;
  std::string file_base_path_;

  // maps from an identifier to a vector of filenames
  std::map<std::string, std::vector<std::string>> audio_file_map_;

  sound_play::SoundClient sound_client_;
  actionlib::SimpleActionClient<sound_play::SoundRequestAction> ac_;


  visualization_msgs::Marker marker_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "play_file_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  TiagoSpeech tiago_speech("bartender_speech_action");
  ros::waitForShutdown();
}
