import numpy as np
import math
import rospy
from phx_rangefinder.msg import ClosestObjectVector
from sensor_msgs.msg import LaserScan


# Input: Number of the scan value, start angle value (rad), angle increment (rad)
# Output: angle of scan value (rad)
def get_Angle(index, angle_min, angle_inc):
    return (angle_min + index * angle_inc)


def callback_ClosestObject(new_LaserScan=LaserScan()):
    data = np.array(new_LaserScan.ranges)

    angle_min = new_LaserScan.angle_min
    angle_inc = new_LaserScan.angle_increment

    # Masking nan values
    masked_data = np.ma.masked_invalid(data).compressed()
    # Indices of min values
    min_indices = np.array(masked_data == masked_data.min()).nonzero()[0]
    #print(min_indices)

    if len(min_indices) >= 2:
        # Mutiple closest object (distances are the same)
        vector_sum = np.zeros(2)
        #print(min_indices)
        for index in min_indices:
            distance = masked_data[index]

            #print(index)
            angle = get_Angle(index, angle_min, angle_inc)
            #print("angle: {}, distance: {}".format(angle, distance))
            result = np.array([distance * math.cos(angle), distance * math.sin(angle)])
            #print(result)
            #print(result.shape)
            #print(vector_sum)
            #print(vector_sum.shape)
            vector_sum += result

        radius = np.sqrt(vector_sum[0]**2 + vector_sum[1]**2)
        #print(vector_sum)
        #print(radius)
        if radius == 0:
            pub_closest_object.publish(0, 0)
        else:
            pub_closest_object.publish(np.arctan2(vector_sum[1], vector_sum[0]), radius)

    else:
        # Only one closest object
        angle = get_Angle(min_indices[0], angle_min, angle_inc)
        pub_closest_object.publish(angle, masked_data[min_indices[0]])


rospy.init_node('Closest_Object_Node')
ros_subscribe_LaserScan = rospy.Subscriber(
    '/scan_filtered', LaserScan, callback_ClosestObject)
pub_closest_object = rospy.Publisher("closest_object", ClosestObjectVector, queue_size=1)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()  # Do not use CPU time when done


ros_subscribe_LaserScan.unregister()
