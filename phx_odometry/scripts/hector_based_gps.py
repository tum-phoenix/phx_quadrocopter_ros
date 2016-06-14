#!/usr/bin/env python

import numpy as np
import rospy

import tf
import sensor_msgs.msg

rospy.init_node('hector_based_odometry')
pub_hector_gps = rospy.Publisher('/phx/hector_gps', sensor_msgs.msg.NavSatFix, queue_size=1)
tf_listener = tf.TransformListener()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    try:
        tf_listener.waitForTransform("map", "scanmatcher_frame", rospy.Time(), rospy.Duration(0.5))
        translation, rotation = tf_listener.lookupTransform("map", "scanmatcher_frame", rospy.Time())
        print 'new transform', translation
        gps_msg = sensor_msgs.msg.NavSatFix()
        gps_msg.latitude = translation[0]
        gps_msg.longitude = translation[1]
        pub_hector_gps.publish(gps_msg)
        print 'published new point'
    except:
        pass
    r.sleep()
