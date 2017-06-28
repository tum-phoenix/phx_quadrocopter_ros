import numpy as np
import scipy.ndimage as nd

import rospy
from sensor_msgs.msg import LaserScan

#Iterations until median is calculated
COUNT = 5

data_2d = np.array([])

def callback_LaserScan_timebased_filter(new_LaserScan=LaserScan()):
    global data_2d
    data = np.array(new_LaserScan.ranges)
    data[np.isnan(data)] = 0
#    print ("Data shape:" , data.shape)

    if data_2d != np.array([]):
        temp = np.append(data_2d, [data], axis=0)
    else: temp = [data]
    data_2d = temp
    print ("Temp shape", np.shape(temp))
    if len(data_2d) == COUNT:
        result = np.median(temp, axis=0)
        #pub_cost_range.publish(result)
        print (result, len(result))
        data_2d = np.array([])




rospy.init_node('LaserScanner_outdoor_timebased_filter')
ros_subscribe_LaserScan = rospy.Subscriber('/scan_filtered', LaserScan, callback_LaserScan_timebased_filter)
pub_cost_range = rospy.Publisher('/scan_filtered_timebased_median', LaserScan, queue_size=1)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()                   # this prevents the node from using 100% CPU



ros_subscribe_LaserScan.unregister()
