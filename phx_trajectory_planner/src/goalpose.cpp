#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "initialpose");

  ros::NodeHandle n;

  geometry_msgs::PoseStamped goal;

  initial.Pose.Point.x = 5;
  initial.Pose.Point.y = 0;
  initial.Pose.Point.z = 0;

  initial.Pose.Quaternion.x = 0;
  initial.Pose.Quaternion.y = 0;
  initial.Pose.Quaternion.z = 0;
  initial.Pose.Quaternion.w = 1;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/phx/current_goal", 1000);

  chatter_pub.publish(goal);

  return 0;

}

