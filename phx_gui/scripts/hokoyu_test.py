from LineScannerPreview import PlaneRadialPlotter3D
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

import geometry_msgs.msg

def callback_LaserScan(new_LaserScan=LaserScan()):
    global plot_output
    print len(new_LaserScan.ranges), new_LaserScan.angle_min, new_LaserScan.angle_max
    data = np.array(new_LaserScan.ranges)[::-1]
    data[data > 6] = 6
    plot_output.set_data(data=data)


pub_frame = rospy.Publisher("laser", tf.msg.tfMessage)
plot_output = PlaneRadialPlotter3D(data_shape=681, start_angle=-2.08621382713, stop_angle=2.08621382713)

rospy.init_node('hokuyo_test')
ros_subscribe_LaserScan = rospy.Subscriber('/scan', LaserScan, callback_LaserScan)

plot_output.run()

ros_subscribe_LaserScan.unregister()

