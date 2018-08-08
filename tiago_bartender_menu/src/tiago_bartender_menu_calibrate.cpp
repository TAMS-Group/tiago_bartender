// #define NDEBUG

#include <ros/ros.h>

#include <eigen3/Eigen/StdVector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include <array>
#include <assert.h>
#include <chrono>
#include <iostream>
#include <random>
#include <unordered_set>
#include <vector>

#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <boost/functional/hash.hpp>

#include <opencv2/opencv.hpp>

//#include <opencv2/persistence.hpp>

#include <opencv2/bgsegm.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/xfeatures2d.hpp>

int main(int argc, char **argv) {

  ros::init(argc, argv, "test", ros::init_options::NoSigintHandler);

  ros::NodeHandle node;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  static cv_bridge::CvImagePtr image_ptr;
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub =
      it.subscribe("/camera/rgb/image_raw", 1,
                   (void (*)(const sensor_msgs::ImageConstPtr &))[](
                       const sensor_msgs::ImageConstPtr &msg) {
                     auto img = cv_bridge::toCvCopy(msg, "bgr8");
                     image_ptr = img;
                     // cv::imshow("received", img->image);
                   });

  while (true) {

    cv::waitKey(1);

    cv::Mat image_scene_color;
    auto img_ptr = image_ptr;
    if (!img_ptr)
      continue;
    image_scene_color = img_ptr->image;

    // cv::resize(image_scene_color, image_scene_color, cv::Size(), 2, 2);

    cv::Mat image_scene;
    cv::cvtColor(image_scene_color, image_scene, CV_BGR2GRAY);

    // cv::imshow("image", image_scene);

    auto chessboard_size = cv::Size(4, 4);

    cv::Mat corners;
    bool found =
        cv::findChessboardCorners(image_scene, chessboard_size, corners);

    if (!found) {
      std::cout << "not found" << std::endl;
    }

    if (found) {
      cv::cornerSubPix(
          image_scene, corners, chessboard_size, cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 300, 0.01));
    }

    cv::Mat image_vis = image_scene_color;
    cv::drawChessboardCorners(image_vis, chessboard_size, corners, found);

    cv::imshow("image", image_vis);

    if (found) {
      /*std::array<cv::Point2f, 4> quad = {
          corners.at<cv::Point2f>(0, 0),
          corners.at<cv::Point2f>(0, chessboard_size.width - 1),
          corners.at<cv::Point2f>(chessboard_size.height - 1,
                                  chessboard_size.width - 1),
          corners.at<cv::Point2f>(chessboard_size.height - 1, 0),
      };*/
      std::array<cv::Point2f, 4> quad = {
          corners.at<cv::Point2f>(0),
          corners.at<cv::Point2f>(3),
          corners.at<cv::Point2f>(12),
          corners.at<cv::Point2f>(15),
      };

      for (size_t i = 0; i < quad.size(); i++)
        std::cout << i << " " << quad[i] << std::endl;

      YAML::Emitter yaml;
      yaml << YAML::BeginSeq;
      for (size_t i = 0; i < quad.size(); i++)
        yaml << quad[i].x << quad[i].y;
      yaml << YAML::EndSeq;
      std::cout << yaml.c_str() << std::endl;

      cv::waitKey();

      std::ofstream resultfile(argv[1]);
      resultfile << yaml.c_str();

      return 0;
    }
  }
}
