#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"

geometry_msgs::PoseStamped initial;
geometry_msgs::PoseStamped goal;

void initialCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  initial.header = msg->header;
  initial.pose = msg->pose;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goal.header = msg->header;
  goal.pose = msg->pose;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "path_planner");

  ros::NodeHandle n1;
  ros::NodeHandle n2;
  ros::NodeHandle n3;

  ros::Subscriber init_sub = n1.subscribe("/phx/pose", 1000, initialCallback);
  ros::Subscriber goal_sub = n2.subscribe("/phx/current_goal", 1000, goalCallback);


  ros::Publisher pathpub = n3.advertise<nav_msgs::Path>("/phx/path", 1000);

  // do shit

  nav_msgs::Path path;
  path.poses[0] = initial;
  path.poses[1] = goal;


  pathpub.publish(path);

  return 0;
}
