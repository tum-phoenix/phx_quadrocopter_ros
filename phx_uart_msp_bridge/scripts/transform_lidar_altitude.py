#!/usr/bin/env python

import numpy as np
import rospy

import tf

import tf2_ros
import geometry_msgs.msg

from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import Attitude

pitch_angle = 0    # rad
roll_angle = 0     # rad#
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
    # q = tf.transformations.quaternion_from_euler(0, 0, 0)
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
    ros_publish_new_altitude.publish(new_msg)
    update_footprint_transform(new_altitude)


def receive_attitude(input_attitude):
    global pitch_angle, roll_angle
    pitch_angle = input_attitude.pitch
    roll_angle = input_attitude.roll

rospy.init_node('transform_lidar_altitude')
ros_subscribe_attitude = rospy.Subscriber('/phx/fc/attitude', Attitude, receive_attitude)
ros_subscribe_altitude = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, convert_altitude_measurement)
ros_publish_new_altitude = rospy.Publisher('/phx/altitude', Altitude, queue_size=1)

r = rospy.Rate(1)
received_map_odom = 0

#  = tf.TransformListener()

while not rospy.is_shutdown():
    """
    try:
        (trans, rot) = tf_listener.lookupTransform('map', 'odom', rospy.Time(0))
        received_map_odom += 1
    except:
        if received_map_odom == 0:
            trans = [0, 0, 0]
        else:
            continue
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "rot_free_odom"
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)
    """
    r.sleep()
