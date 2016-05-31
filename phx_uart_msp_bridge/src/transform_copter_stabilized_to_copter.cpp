#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf/tf.h"
#include <math.h>               // for M_PI
#include "sensor_msgs/Imu.h"
#include "phx_uart_msp_bridge/Attitude.h"

// predefine callback function names
void imu_callback(const sensor_msgs::Imu::ConstPtr&);

// define variables used throughout the whole script
boost::shared_ptr<tf2_ros::TransformBroadcaster> TF_publisher;

geometry_msgs::Transform tf_copter_stabilized_to_copter;
geometry_msgs::TransformStamped tf_stamped_copter_stabilized_to_copter;

uint32_t received_imu = 0;


// main function is the code which will be executed
int main(int argc, char** argv)
{
    // std::cout << "\033[1;31m>>> starting TF copter -> copter_stabilized \033[0m"<< std::endl;
    ros::init(argc, argv, "transform_copter_stabilized_to_copter");
    ros::NodeHandle ROS_handle;
    ros::Rate loop_rate(10);    // Hz

    // define subscribers on the according topics linking them to a callback function
    // ros::Subscriber imu_subscriber = ROS_handle.subscribe<sensor_msgs::Imu>("phx/imu", 1, imu_callback);
    ros::Subscriber attitude_subscriber = ROS_handle.subscribe<phx_uart_msp_bridge::Attitude>("phx/fc/attitude", 1, attitude_callback);
    TF_publisher.reset(new tf2_ros::TransformBroadcaster());

    // main loop -------------------------------------------------------------------------------------------
    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}



void imu_callback(const sensor_msgs::Imu::ConstPtr& input_imu) {
    // std::cout << "\033[1;31m>>> IMU input is used for TF copter -> copter_stabilized \033[0m"<< std::endl;

    //  copter_stabilized -> copter
    tf_copter_stabilized_to_copter.translation.x = 0;
    tf_copter_stabilized_to_copter.translation.y = 0;
    tf_copter_stabilized_to_copter.translation.z = 0;
    tf_copter_stabilized_to_copter.rotation = input_imu->orientation;

    tf_stamped_copter_stabilized_to_copter.header.stamp = ros::Time::now();
    tf_stamped_copter_stabilized_to_copter.header.seq = received_imu;
    tf_stamped_copter_stabilized_to_copter.header.frame_id = "copter_stabilized";
    tf_stamped_copter_stabilized_to_copter.child_frame_id = "copter";

    tf_stamped_copter_stabilized_to_copter.transform = tf_copter_stabilized_to_copter;
    TF_publisher->sendTransform(tf_stamped_copter_stabilized_to_copter);

    received_imu++;
}



void attitude_callback(const phx_uart_msp_bridge::Attitude::ConstPtr& input_attitude) {
    // std::cout << "\033[1;31m>>> ATTITUDE input is used for TF copter -> copter_stabilized \033[0m"<< std::endl;

    //  copter_stabilized -> copter
    tf_copter_stabilized_to_copter.translation.x = 0;
    tf_copter_stabilized_to_copter.translation.y = 0;
    tf_copter_stabilized_to_copter.translation.z = 0;
    tf_copter_stabilized_to_copter.rotation = tf::createQuaternionMsgFromRollPitchYaw(((float) input_attitude->roll / 360. * 2*M_PI),
                                                                                      ((float) input_attitude->pitch / 360. * 2*M_PI),
                                                                                      ((float) 0));

    tf_stamped_copter_stabilized_to_copter.header.stamp = ros::Time::now();
    tf_stamped_copter_stabilized_to_copter.header.seq = received_imu;
    tf_stamped_copter_stabilized_to_copter.header.frame_id = "copter_stabilized";
    tf_stamped_copter_stabilized_to_copter.child_frame_id = "copter";

    tf_stamped_copter_stabilized_to_copter.transform = tf_copter_stabilized_to_copter;
    TF_publisher->sendTransform(tf_stamped_copter_stabilized_to_copter);

    received_imu++;
}








