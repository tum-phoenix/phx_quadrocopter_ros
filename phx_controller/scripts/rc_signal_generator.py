#!/usr/bin/env python
import numpy as np
import time

import rospy
from sensor_msgs.msg import Joy


class RCgen:
    def __init__(self):
        rospy.init_node('rc_signal_generator')
        self.pub = rospy.Publisher('/phx/fc/rc_computer', Joy, queue_size=1)

        self.freq = 20     # Hz
        self.r = rospy.Rate(self.freq)

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

            joy_msg = Joy()

            yaw = 1000 + np.sin(time.time() + 0) * 400
            roll = 1000 + np.sin(time.time() + 1) * 400
            pitch = 1000 + np.sin(time.time() + 2) * 200
            throttle = 1000 + np.sin(time.time() + 3) * 400

            aux1 = 1100
            aux2 = 1200
            aux3 = 1300
            aux4 = 1400
            # Replay and override current rc
            joy_msg.axes = [aux1, aux2, aux3, aux4]
            joy_msg.buttons = [yaw, roll, pitch, throttle]
            print 'updating'
            self.pub.publish(joy_msg)


if __name__ == '__main__':
    try:
        controller_node = RCgen()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass