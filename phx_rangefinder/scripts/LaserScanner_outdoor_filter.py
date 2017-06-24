#!/usr/bin/env python
import numpy as np
import scipy.ndimage as nd

import rospy
from sensor_msgs.msg import LaserScan


def callback_LaserScan(new_LaserScan=LaserScan()):
    data = nd.filters.median_filter(np.array(new_LaserScan.ranges), 10)
    new_LaserScan.ranges = data
    pub_cost_range.publish(new_LaserScan)

rospy.init_node('LaserScanner_outdoor_filter')
ros_subscribe_LaserScan = rospy.Subscriber('/scan_filtered', LaserScan, callback_LaserScan)
pub_cost_range = rospy.Publisher('/scan_filtered_median', LaserScan, queue_size=10)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()                   # this prevents the node from using 100% CPU



ros_subscribe_LaserScan.unregister()

