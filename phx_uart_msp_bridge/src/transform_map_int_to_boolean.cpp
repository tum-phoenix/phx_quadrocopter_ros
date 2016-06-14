#include "ros/ros.h"

int main() {

    ros::init(argc, argv, "transform_map_int_to_boolean");
        ros::NodeHandle ROS_handle;
        ros::Rate loop_rate(10);    // Hz


    //Subsricber for the old integer map
    ros::Subscriber attitude_subscriber = ROS_handle.subscribe<nav_msgs::OccupancyGrid>("map", 1, map_callback);


    //Publisher for the new boolean map
    ros::Publisher map_boolean_pub = n.advertise<phy_uart_msp_bridge::BooleanMap>("map_boolean", 1);


    TF_publisher.reset(new tf2_ros::TransformBroadcaster());

    // main loop -------------------------------------------------------------------------------------------
    while (ros::ok())
    {
        ros::spin();

        phx_uart_msp_bridge::BooleanMap booleanMapMsg;

        headerMsg.seq = received_imu;       //Received_imu?
        headerMsg.stamp = ros::Time::now();
        headerMsg.frame_id = "???";
        booleanMapMsg.header = headerMsg;

        //TODO: Create the bool[] part of the booleanMapMsg

        map_boolean_pub.publish(booleanMapMsg);     //Published the new booleanMap message
    }
    return 0;
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& input_map) {

}