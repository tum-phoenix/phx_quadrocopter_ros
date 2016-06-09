import time
import numpy as np

import rospy
from phx_uart_msp_bridge.msg import Diagnostics


rospy.init_node('diagnostics_gui_test')

ros_publish_diagnostics = rospy.Publisher('/diag_out', Diagnostics, queue_size=1)

# initialize 'speed'-limit for endless loop
r = rospy.Rate(1)


# start endless loop until rospy.is_shutdown()
while not rospy.is_shutdown():
    m = Diagnostics()
    m.header.stamp.secs = rospy.get_time()
    m.val_a0 = 0
    m.val_a1 = np.random.normal()
    m.val_a2 = 1
    m.val_b0 = np.sin(time.time())
    m.val_b1 = np.random.normal()
    m.val_b2 = np.cos(time.time())
    m.val_c0 = 0
    m.val_c1 = np.cos(2.*time.time())
    m.val_c2 = 1

    ros_publish_diagnostics.publish(m)

    print 'published diagnostics'

    r.sleep()       # this prevents the node from using 100% CPU