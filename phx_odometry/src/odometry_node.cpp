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
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    
    if(dt<0.2){
        //ROS_INFO("dt: %f", dt);
        //ROS_INFO("acc: %f, %f, %f", imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
        
        vx = 0.9*vx+imu_msg->linear_acceleration.x*dt;
        vy = 0.9*vy+imu_msg->linear_acceleration.y*dt;
        vz = 0.9*vz+imu_msg->linear_acceleration.z*dt;
        
        //ROS_INFO("vel: %f, %f, %f", vx, vy, vz);
        
        x = x+vx*dt;
        y = y+vy*dt;
        z = z+vz*dt;
        
        if ((x != x || y != y || z != z)) {
            ROS_WARN("NAN %f, %f, %f", x, y, z);
        }
        
        //ROS_INFO("pos: %f, %f, %f", x, y, z);
    } else {
        ROS_WARN("dt: %f", dt);
    }

    geometry_msgs::Transform transform;
    
    if (!(x != x || y != y || z != z)) {
        transform.translation.x=x;
        transform.translation.y=y;
        transform.translation.z=z;
    } else {
        ROS_WARN("Translation NaN");
        ROS_WARN("%f, %f, %f", x, y, z);

    }
    
    if (!(isnan(imu_msg->orientation.x) || isnan(imu_msg->orientation.y) || isnan(imu_msg->orientation.x)|| isnan(imu_msg->orientation.w))){
        transform.rotation.x=imu_msg->orientation.x;
        transform.rotation.y=imu_msg->orientation.y;
        transform.rotation.z=imu_msg->orientation.z;
        transform.rotation.w=imu_msg->orientation.w;
    } else {
        ROS_WARN("Orientation NaN");
        ROS_WARN("%f, %f, %f, %f", imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);

    }
    
    transformStamped.header.stamp=current_time;
    transformStamped.header.frame_id="world";
    transformStamped.child_frame_id="drone";
    transformStamped.transform=transform;
    transformBroadcaster->sendTransform(transformStamped);
    
    /*nav_msgs::Odometry odometry;
    odometry.header.stamp = current_time;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "drone";
    if (!(isnan(x) || isnan(y) || isnan(z))) {
        //update only if not Nan
        //odometry.pose.pose.position.x = x;
        //odometry.pose.pose.position.y = y;
        //odometry.pose.pose.position.z = z;
    } else {
        ROS_WARN("Odometry NaN");
        ROS_INFO("%f, %f, %f", x, y, z);

    }


    odometry.pose.pose.orientation = imu_msg->orientation;
    odometry.twist.twist.linear.x = vx;
    odometry.twist.twist.linear.y = vy;
    odometry.twist.twist.linear.z = vz;
    odometryPublisher.publish(odometry);*/
    
    last_time=current_time;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle n;
    ros::Subscriber imuSubscriber = n.subscribe("/phoenix/imu", 1000, imuCallback);
    odometryPublisher = n.advertise<nav_msgs::Odometry>("/phoenix/odom", 50);
    transformBroadcaster.reset(new tf2_ros::TransformBroadcaster());
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    
    ros::spin();
    
    return 0;
}

