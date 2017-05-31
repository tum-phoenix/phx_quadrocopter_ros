#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "initialpose");

  ros::NodeHandle n;

  geometry_msgs::PoseStamped goal;

  goal.pose.position.x = 5;
  goal.pose.position.y = 0;
  goal.pose.position.z = 0;

  goal.pose.orientation.x = 0;
  goal.pose.orientation.y = 0;
  goal.pose.orientation.z = 0;
  goal.pose.orientation.w = 1;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/phx/current_goal", 1000);

  chatter_pub.publish(goal);

  return 0;

}

