#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"

geometry_msgs::PoseStamped initial;
geometry_msgs::PoseStamped goal;

void initialCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  initial.Pose.Point.x = msg.Pose.Point.x;
  initial.Pose.Point.y = msg.Pose.Point.y;
  initial.Pose.Point.z = msg.Pose.Point.z;

  initial.Pose.Quaternion.x = msg.Pose.Quaternion.x;
  initial.Pose.Quaternion.y = msg.Pose.Quaternion.y;
  initial.Pose.Quaternion.z = msg.Pose.Quaternion.z;
  initial.Pose.Quaternion.w = msg.Pose.Quaternion.w;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goal.Pose.Point.x = msg.Pose.Point.x;
  goal.Pose.Point.y = msg.Pose.Point.y;
  goal.Pose.Point.z = msg.Pose.Point.z;

  goal.Pose.Quaternion.x = msg.Pose.Quaternion.x;
  goal.Pose.Quaternion.y = msg.Pose.Quaternion.y;
  goal.Pose.Quaternion.z = msg.Pose.Quaternion.z;
  goal.Pose.Quaternion.w = msg.Pose.Quaternion.w;
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
