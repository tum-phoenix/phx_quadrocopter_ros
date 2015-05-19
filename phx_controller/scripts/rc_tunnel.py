#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy


class ControllerNode():
    def __init__(self):
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.sub = rospy.Subscriber('/phx/rc_marvic', Joy, self.rcCallback)
        self.pub = rospy.Publisher('/phx/rc_computer', Joy)

        self.freq = 100     #Hz
        r = rospy.Rate(self.freq)
        self.counter = 0
        
        while not rospy.is_shutdown():
            r.sleep()

    def rcCallback(self, joy_msg):
        self.counter += 1

        if self.counter % 2 == 0:
            joy_msg = Joy()

            # Replay and override current rc
            joy_msg.axes = self.input_rc[0:4]
            joy_msg.buttons = self.input_rc[4:]
            self.input_rc[0] = joy_msg.axes[0]
            self.input_rc[1] = joy_msg.axes[1]
            self.input_rc[2] = joy_msg.axes[2]
            self.input_rc[3] = joy_msg.axes[3]
            self.input_rc[4] = joy_msg.buttons[0]
            self.input_rc[5] = joy_msg.buttons[1]
            self.input_rc[6] = joy_msg.buttons[2]
            self.input_rc[7] = joy_msg.buttons[3]


if __name__ == '__main__':
    rospy.init_node('controller')
    try:
        controller_node = ControllerNode()
    except rospy.ROSInterruptException:
        pass