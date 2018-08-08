#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <tiago_bartender_speech/FileName.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

class TiagoSpeech
{
public:
  TiagoSpeech()
  {
    file_base_path_ = ros::package::getPath("tiago_bartender_speech");
    file_base_path_ = file_base_path_ + "/audio_files/";
    play_file_service_ = nh_.advertiseService("play_audo_file", &TiagoSpeech::play_file, this);
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("sound_marker",0);
  }
private:
  bool play_file(tiago_bartender_speech::FileName::Request& req, tiago_bartender_speech::FileName::Response& res)
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
    marker.text = req.file_name;
    vis_pub_.publish(marker);

    sound_client_.playWave(file_base_path_ + req.file_name);

    marker.action = visualization_msgs::Marker::DELETE;
    vis_pub_.publish(marker);
    return true;
  }

  ros::NodeHandle nh_;
  ros::ServiceServer play_file_service_;
  ros::Publisher vis_pub_;
  std::string file_base_path_;

  sound_play::SoundClient sound_client_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "play_file_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  TiagoSpeech tiago_speech;
  while(ros::ok())
  {}
}
