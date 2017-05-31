#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "initialpose");

  ros::NodeHandle n;

  geometry_msgs::PoseStamped initial;

  initial.Pose.Point.x = 2;
  initial.Pose.Point.y = 0;
  initial.Pose.Point.z = 1;

  initial.Pose.Quaternion.x = 0;
  initial.Pose.Quaternion.y = 0;
  initial.Pose.Quaternion.z = 0;
  initial.Pose.Quaternion.w = 1;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/phx/pose", 1000);

  chatter_pub.publish(initial);

  return 0;

}

