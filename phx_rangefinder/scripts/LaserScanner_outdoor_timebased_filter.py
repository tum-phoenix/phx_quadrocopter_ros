import numpy as np
import scipy.ndimage as nd

import rospy
from sensor_msgs.msg import LaserScan

#Iterations until median is calculated
COUNT = 5

data_2d = []

def callback_LaserScan_timebased_filter(new_LaserScan=LaserScan()):
    data = np.array(new_LaserScan.ranges)
    data_2d.append(data)
    if len(data_2d) == COUNT:
        data = nd.filters.median_filter(np.array(new_LaserScan.ranges), axis=0)
        pub_cost_range.publish(data)
        data = []



rospy.init_node('LaserScanner_outdoor_timebased_filter')
ros_subscribe_LaserScan = rospy.Subscriber('/scan_filtered', LaserScan, callback_LaserScan_timebased_filter)
pub_cost_range = rospy.Publisher('/scan_filtered_timebased_median', LaserScan, queue_size=1)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()                   # this prevents the node from using 100% CPU



ros_subscribe_LaserScan.unregister()
