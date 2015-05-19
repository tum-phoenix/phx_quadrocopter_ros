#!/usr/bin/env python
import time
import rospy
import numpy as np
from phx_arduino_uart_bridge.msg import Altitude
from sensor_msgs.msg import Joy



class ControllerNode():
    def __init__(self):
        
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.sub = rospy.Subscriber('/phx/rc_marvic', Joy, self.rcCallback)
        self.sub = rospy.Subscriber('/phx/altitude_multiwii', AltitudeMessage, self.altitudeCallback)
        self.pub = rospy.Publisher('/phx/rc_computer', Joy)

        self.freq = 100 #hz
        r = rospy.Rate(self.freq)
        
        while not rospy.is_shutdown():
            r.sleep()

    def altitudeCallback(self, altitude_msg):

        self.setPoint = 150
        self.p = 10
        controlCommand = (setPoint - altitude_msg.estimated_altitude) * self.p

        joy_msg = Joy()

        # Replay and override current rc
        joy_msg.axis[0] = self.input_rc[0]
        joy_msg.axis[1] = self.input_rc[1]
        joy_msg.axis[2] = self.input_rc[2]
        joy_msg.axis[3] = controlCommand
        joy_msg.buttons[0] = self.input_rc[4]
        joy_msg.buttons[1] = self.input_rc[5]
        joy_msg.buttons[2] = self.input_rc[6]
        joy_msg.buttons[3] = self.input_rc[7]
        
        self.pub.publish(joy_msg)

        
    def rcCallback(self, joy_msg):
        
        self.input_rc[0] = joy_msg.axis[0]
        self.input_rc[1] = joy_msg.axis[1]
        self.input_rc[2] = joy_msg.axis[2]
        self.input_rc[3] = joy_msg.axis[3] # Throttle
        self.input_rc[4] = joy_msg.buttons[0]
        self.input_rc[5] = joy_msg.buttons[1]
        self.input_rc[6] = joy_msg.buttons[2]
        self.input_rc[7] = joy_msg.buttons[3]


if __name__ == '__main__':
    rospy.init_node('controller')
    try:
        controller_node = ControllerNode()
    except rospy.ROSInterruptException: pass