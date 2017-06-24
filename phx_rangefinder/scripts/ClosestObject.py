import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan

def callback_ClosestObject(new_LaserScan=LaserScan()):
    data = np.array(new_LaserScan.ranges)
    angle_min = new_LaserScan.angle_min
    angle_inc = new_LaserScan.angle_increment

    # Masking nan values
    masked_data = np.ma.masked_invalid(data)
    # Indices of min values
    min_indices = np.ma.where(masked_data == masked_data.min())

    for x in np.nditer(min_indices):
        # Calculate angles
        angle = math.degrees(angle_min + x*angle_inc)
        print("Min value", data[x], "Angle", angle)


rospy.init_node('Closest_Object_Node')
ros_subscribe_LaserScan = rospy.Subscriber('/scan_filtered', LaserScan, callback_ClosestObject)
pub_closest_object = rospy.Publisher("closest_object", LaserScan, queue_size=1)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep() # Do not use CPU time when done


ros_subscribe_LaserScan.unregister()
