import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan


# Input: Number of the scan value, start angle value (rad), angle increment (rad)
# Output: angle of scan value (rad)
def get_Angle(index, angle_min, angle_inc):
    return angle_min + index * angle_inc


def callback_ClosestObject(new_LaserScan=LaserScan()):
    data = np.array(new_LaserScan.ranges)
    angle_min = new_LaserScan.angle_min
    angle_inc = new_LaserScan.angle_increment

    # Masking nan values
    masked_data = np.ma.masked_invalid(data)
    # Indices of min values
    min_indices = np.ma.where(masked_data == masked_data.min())

    if len(min_indices) >= 1:
        # Mutiple closest object (distances are the same)
        vector_sum = np.array(0, 0)
        for index in min_indices:
            distance = data[index]
            angle = get_Angle(index, angle_min, angle_inc)
            result = np.array(distance * math.cos(angle), distance * math.sin(angle))
            vector_sum += result

        radius = np.sqrt(vector_sum[0]**2 + vector_sum[0]**2)
        if radius == 0:
            pub_closest_object.publish(0, 0)
        else:
            pub_closest_object.publish(np.arctan2(vector_sum[1], vector_sum[0]), radius)

    else:
        # Only one closest object
        angle = get_Angle(min_indices[0], angle_min, angle_inc)
        pub_closest_object.publish(angle, data[angle_min[0]])

    for x in np.nditer(min_indices):
        # Calculate angles
        angle = math.degrees(angle_min + x * angle_inc)
        print("Min value", data[x], "Angle", angle)
        pub_closest_object.publish(angle, data[x], )


rospy.init_node('Closest_Object_Node')
ros_subscribe_LaserScan = rospy.Subscriber(
    '/scan_filtered', LaserScan, callback_ClosestObject)
pub_closest_object = rospy.Publisher("closest_object", LaserScan, queue_size=1)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()  # Do not use CPU time when done


ros_subscribe_LaserScan.unregister()
