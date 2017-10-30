#!/usr/bin/env python

import numpy as np          # mathematics

import rospy                # ROS interface
import tf2_ros              # ROS transformation support

# ROS message types used throughout this script
import geometry_msgs.msg
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import Attitude

# initial definition of globally used variables
pitch_angle = 0                         # in rad
roll_angle = 0                          # in rad
br = tf2_ros.TransformBroadcaster()


def update_footprint_transform(height):
    global br
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "footprint"
    t.child_frame_id = "copter_stabilized"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = height
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1

    br.sendTransform(t)


def convert_altitude_measurement(input_altitude):
    global pitch_angle, roll_angle
    new_altitude = 1.0 * input_altitude.estimated_altitude * np.cos(pitch_angle/180*np.pi) * np.cos(roll_angle/180*np.pi)

    new_msg = Altitude()
    new_msg.estimated_altitude = new_altitude
    new_msg.header.stamp = rospy.Time.now()
    ros_publish_new_altitude.publish(new_msg)

    # timestamp kommt in Message nicht an --> sollte diese Zeile nicht vor dem publish stehen?
    update_footprint_transform(new_altitude)


def receive_attitude(input_attitude):
    global pitch_angle, roll_angle
    pitch_angle = input_attitude.pitch
    roll_angle = input_attitude.roll


# initialize node
rospy.init_node('transform_lidar_altitude')

# listening for Attitude msg on /phx/fc/attitude topic
ros_subscribe_attitude = rospy.Subscriber('/phx/fc/attitude', Attitude, receive_attitude)

# listening for Altitude msg on /phx/marvicAltitude/altitude topic
ros_subscribe_altitude = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, convert_altitude_measurement)

# publishing Altitude messages on /phx/altitude topic
ros_publish_new_altitude = rospy.Publisher('/phx/altitude', Altitude, queue_size=1)

# initialize 'speed'-limit for endless loop
r = rospy.Rate(1)


# start endless loop until rospy.is_shutdown()
while not rospy.is_shutdown():
    r.sleep()                   # this prevents the node from using 100% CPU
