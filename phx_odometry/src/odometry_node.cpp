#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

boost::shared_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;
ros::Publisher odometryPublisher;
geometry_msgs::TransformStamped transformStamped;
ros::Time current_time, last_time;

double x = 0.0;
double y = 0.0;
double z = 0.2;

double vx = 0.0;
double vy = 0.0;
double vz = 0.0;

double ax = 0.0;
double ay = 0.0;
double az = 0.0;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{

    geometry_msgs::Transform transform;
    transform.rotation.x=imu_msg->orientation.x;
    transform.rotation.y=imu_msg->orientation.y;
    transform.rotation.z=imu_msg->orientation.z;
    transform.rotation.w=imu_msg->orientation.w;
    transformStamped.header.stamp=current_time;
    transformStamped.header.frame_id="world";
    transformStamped.child_frame_id="drone";
    transformStamped.transform=transform;
    transformBroadcaster->sendTransform(transformStamped);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle n;
    ros::Subscriber imuSubscriber = n.subscribe("/phoenix/stat_imu", 5, imuCallback);
    odometryPublisher = n.advertise<nav_msgs::Odometry>("/phoenix/odom", 50);
    transformBroadcaster.reset(new tf2_ros::TransformBroadcaster());
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    
    ros::spin();
    
    return 0;
}

