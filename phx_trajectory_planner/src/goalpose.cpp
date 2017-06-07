#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "goalpose");

  ros::NodeHandle n;

  geometry_msgs::PoseStamped goal;

  goal.pose.position.x = 5;
  goal.pose.position.y = 0;
  goal.pose.position.z = 0;

  goal.pose.orientation.x = 0;
  goal.pose.orientation.y = 0;
  goal.pose.orientation.z = 0;
  goal.pose.orientation.w = 1;

  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();

  ros::Rate loop_rate(10);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/phx/current_goal", 1000);

  while (ros::ok()){
    chatter_pub.publish(goal);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;

}

