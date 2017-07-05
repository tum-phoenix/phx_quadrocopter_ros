import numpy as np
import scipy.ndimage as nd

import rospy
from sensor_msgs.msg import LaserScan

# Counter for updating data array with round robin scheduling
COUNT = 0

# Size of array. Increase for better median filtering
SIZE = 5

# Array of range arrays
data_2d = np.array([])

def callback_LaserScan_timebased_filter(new_LaserScan=LaserScan()):
    global data_2d
    global COUNT

    data = np.array(new_LaserScan.ranges)
    # Set invalid values to zero
    data[np.isnan(data)] = 0

    if not data_2d:
        data_2d = numpy.array([data]*SIZE)
    
    print ("Temp shape", np.shape(temp))
    if COUNT => SIZE: COUNT = 0
    data_2d[COUNT] = data
    COUNT += 1
    result = np.median(data_2d, axis=0)
    
    print (result, len(result))
    
    # Publish filtered ranges
    new_LaserScan.ranges = result
    pub_scan_filtered.publish(new_LaserScan)




rospy.init_node('LaserScanner_outdoor_timebased_filter')
ros_subscribe_LaserScan = rospy.Subscriber('/scan_filtered', LaserScan, callback_LaserScan_timebased_filter)
pub_cost_range = rospy.Publisher('/scan_filtered_timebased_median', LaserScan, queue_size=1)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()                   # this prevents the node from using 100% CPU



ros_subscribe_LaserScan.unregister()
