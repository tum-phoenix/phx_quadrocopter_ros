#!/usr/bin/env python
import numpy as np
import time

import rospy
from phx_uart_msp_bridge.msg import RemoteControl


class RCgen:
    def __init__(self):
        rospy.init_node('rc_signal_generator')
        self.pub = rospy.Publisher('/phx/fc/rc_computer', RemoteControl, queue_size=1)

        self.freq = 20     # Hz
        self.r = rospy.Rate(self.freq)

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

            rc_msg = RemoteControl()

            yaw = 1500 + np.sin(time.time() + 0) * 400
            roll = 1500 + np.sin(time.time() + 1) * 400
            pitch = 1500 + np.sin(time.time() + 2) * 200
            throttle = 1500 + np.sin(time.time() + 3) * 400

            aux1 = 1100 #+ np.sin(time.time() + 3) * 400
            aux2 = 1200 #+ np.sin(time.time() + 3) * 400
            aux3 = 1300 #+ np.sin(time.time() + 3) * 400
            aux4 = 1400
            # Replay and override current rc
            rc_msg.pitch = pitch
            rc_msg.roll = roll
            rc_msg.yaw = yaw
            rc_msg.throttle = throttle
            rc_msg.aux1 = aux1
            rc_msg.aux2 = aux2
            rc_msg.aux3 = aux3
            rc_msg.aux4 = aux4

            print 'updating'
            self.pub.publish(rc_msg)


if __name__ == '__main__':
    try:
        controller_node = RCgen()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
