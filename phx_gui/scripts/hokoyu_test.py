from LineScannerPreview import LineScannerPreview
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan


def callback_LaserScan(new_LaserScan=LaserScan()):
    global plot_output, old_data
    print len(new_LaserScan.ranges)
    data = np.array(new_LaserScan.ranges)
    data[np.isnan(data)] = old_data[np.isnan(data)]
    plot_output.add_new_data(new_data=data)
    old_data = data

plot_output = LineScannerPreview(log_length=100, data_shape=681)

rospy.init_node('hokuyo_test')
ros_subscribe_LaserScan = rospy.Subscriber('/scan', LaserScan, callback_LaserScan)
old_data = np.zeros((681))

plot_output.run()

ros_subscribe_LaserScan.unregister()