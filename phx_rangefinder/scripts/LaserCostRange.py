import numpy as np
import scipy

import rospy
from sensor_msgs.msg import LaserScan


def callback_LaserScan(new_LaserScan=LaserScan()):
    print len(new_LaserScan.ranges), new_LaserScan.angle_min, new_LaserScan.angle_max
    data = np.array(new_LaserScan.ranges)
    cost = np.zeros_like(data)

    # calculate cost
    cost[data > 10] = 0
    cost[data < 10] = 1.0 * (10 - data[data < 10]) / 10
    cost[data < 2] = 1.

    # smooth sharp edges
    kernel = np.zeros(15)
    kernel[5:-5] = 1
    kernel /= np.sum(kernel)

    convolved_cost = scipy.convolve(cost, kernel)

    cost[data < 5] = convolved_cost[data < 5]

    cost *= 5
    new_LaserScan.ranges = cost
    pub_cost_range.publish(new_LaserScan)

    print 'published data 2', np.mean(cost)



rospy.init_node('Laser_Cost_Range_Node')
ros_subscribe_LaserScan = rospy.Subscriber('/scan_filtered', LaserScan, callback_LaserScan)
pub_cost_range = rospy.Publisher("cost_ranges", LaserScan, queue_size=10)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()                   # this prevents the node from using 100% CPU



ros_subscribe_LaserScan.unregister()

