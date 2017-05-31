#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "initialpose");

  ros::NodeHandle n;

  geometry_msgs::PoseStamped initial;
  initial.pose.position.x = 2;
  initial.pose.position.y = 0;
  initial.pose.position.z = 1;

  initial.pose.orientation.x = 0;
  initial.pose.orientation.y = 0;
  initial.pose.orientation.z = 0;
  initial.pose.orientation.w = 1;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/phx/pose", 1000);

  chatter_pub.publish(initial);

  return 0;

}

