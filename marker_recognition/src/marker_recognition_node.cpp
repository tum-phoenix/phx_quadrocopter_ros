#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>

#include<ros/ros.h>
#include<ros/package.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>

#include "marker_recognition/SignRecognition.h"

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  SignRecognition* recognizer_;

public:
  ImageConverter(SignRecognition* recognizer):
    nh_(),
    it_(nh_),
    recognizer_(recognizer)
  {
    //Subscribe to video feeds
    image_sub_ = it_.subscribe("/cam_bottom/image_rect",1,&ImageConverter::imageCb,this);
  }

  ~ImageConverter()
  {}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    recognizer_->updateImage(cv_ptr->image);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"marker_recognition");
  ros::NodeHandle nh("~");

  // need this to give gdb time to attach
  ros::Duration(1.0).sleep();

  std::vector<std::string> reference_images;
  std::string images_param;
  if (nh.getParam("reference_images", images_param))
  {
    std::stringstream ss(images_param);
    std::string buffer;
    while (ss >> buffer)
      reference_images.push_back(buffer);
  }
  else
  {
    ROS_ERROR("No reference images provided!");
    reference_images.push_back(ros::package::getPath("marker_recognition")+"/images/Star.jpg");
  }

  std::string camera_matrix;
  if (!nh.getParam("camera_matrix", camera_matrix))
    ROS_ERROR("No camera matrix provided");

  // 0: use ORB, 1: use shape matching
  int method = 0;
  if (!nh.getParam("recognition_method", method))
  {
    ROS_ERROR("No recognition method provided");
    method = 0;
  }

  SignRecognition main_recognizer(reference_images, camera_matrix, method);
  ImageConverter ic(&main_recognizer);
  ros::spin();
  return 0;
}

