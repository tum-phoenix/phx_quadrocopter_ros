#!/usr/bin/env python
import numpy as np          # mathematics
import math

import rospy                # ROS interface
import tf2_ros              # ROS transformation support

# ROS message types used throughout this script
import geometry_msgs.msg
import sensor_msgs.msg

target_pos = np.zeros((3))
cost_ranges = None


def receive_new_target(input_point):
    # print 'new point:', input_point
    global target_pos
    target_pos[0] = input_point.point.x
    target_pos[1] = input_point.point.y
    target_pos[2] = input_point.point.z


def receive_new_cost_scan(input_scan):
    global cost_ranges
    print 'new ranges'
    cost_ranges = input_scan

# initial definition of globally used variables
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# initialize node
rospy.init_node('position_hold_node')

# listening for Attitude msg on /phx/fc/attitude topic
ros_subscribe_target_position = rospy.Subscriber('/clicked_point', geometry_msgs.msg.PointStamped, receive_new_target)
ros_subscribe_cost_map = rospy.Subscriber('/cost_ranges', sensor_msgs.msg.LaserScan, receive_new_cost_scan)

# initialize 'speed'-limit for endless loop
r = rospy.Rate(1)


# start endless loop until rospy.is_shutdown()
while not rospy.is_shutdown():
    try:
        trans = tfBuffer.lookup_transform('map', 'footprint', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue
    print 'current position:', trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
    print 'current target:', target_pos[0], target_pos[1], target_pos[2]
    print 'current transform:', trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
    r.sleep()       # this prevents the node from using 100% CPU
