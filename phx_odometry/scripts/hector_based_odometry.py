#!/usr/bin/env python

import numpy as np
import rospy

import tf

import tf2_ros
import geometry_msgs.msg


def update_odometry_frame(x, y, z):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "map"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1

    br.sendTransform(t)


rospy.init_node('hector_based_odometry')
tf_listener = tf.TransformListener()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    try:
        tf_listener.waitForTransform("map", "scanmatcher_frame", rospy.Time(), rospy.Duration(0.5))
        translation, rotation = tf_listener.lookupTransform("map", "scanmatcher_frame", rospy.Time())
        update_odometry_frame(-translation[0], -translation[1], translation[2])
    except:
        pass
    r.sleep()
