#!/usr/bin/env python
import numpy as np
import time

import rospy
from geometry_msgs.msg import Twist


class CmdVel:
    def __init__(self):
        rospy.init_node('cmd_vel_generator')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.freq = 20     # Hz
        self.r = rospy.Rate(self.freq)

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

            vel_msg = Twist()

            vel_msg.linear.x = 3 * np.random.random()
            vel_msg.linear.y = 3 * np.random.random()
            vel_msg.linear.z = 3 * np.random.random()

            vel_msg.angular.x = np.random.random()
            vel_msg.angular.y = np.random.random()
            vel_msg.angular.z = np.random.random()

            print 'updating'
            self.pub.publish(vel_msg)


if __name__ == '__main__':
    try:
        controller_node = CmdVel()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
