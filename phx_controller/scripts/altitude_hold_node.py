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
        self.sub = rospy.Subscriber('/phx/altitude_marvic', Altitude, self.altitudeCallback)
        self.pub = rospy.Publisher('/phx/rc_computer', Joy, queue_size=1)

        self.setPoint = 150

        self.p = 1
        self.d = 4
        self.setPoint_d = 0
        self.i = 0
        self.sum_i = 0
        self.i_stop = 100
        self.controlCommand = 1000

        self.freq = 100     # Hz
        self.r = rospy.Rate(self.freq)

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def altitudeCallback(self, altitude_msg):
        if self.input_rc[4] > 1500:
            self.setPoint = altitude_msg.estimated_altitude
            self.controlCommand = self.input_rc[3]

        self.sum_i += self.setPoint - altitude_msg.estimated_altitude
        if self.sum_i >= self.i_stop:
            self.sum_i = self.i_stop
        elif self.sum_i <= -self.i_stop:
            self.sum_i = -self.i_stop
        controlCommand_p = (self.setPoint - altitude_msg.estimated_altitude) * self.p
        controlCommand_d = (self.setPoint_d - altitude_msg.variation) * self.d
        controlCommand_i = self.sum_i * self.i
        un_cliped = self.controlCommand + controlCommand_p + controlCommand_d + controlCommand_i
        self.controlCommand = np.clip(un_cliped, 1000, 2000)
        joy_msg = Joy()

        # Replay and override current rc
        joy_msg.axes = self.input_rc[0:3] + [self.controlCommand]
        joy_msg.buttons = self.input_rc[4:]
        
        self.pub.publish(joy_msg)
        print 'set_point:', self.setPoint, '\t alt:', altitude_msg.estimated_altitude, '\t controlCommand', un_cliped, self.controlCommand, 'p:', controlCommand_p, 'i:', controlCommand_i, 'd:', controlCommand_d

    def rcCallback(self, joy_msg):
        self.input_rc[0] = joy_msg.axes[0]
        self.input_rc[1] = joy_msg.axes[1]
        self.input_rc[2] = joy_msg.axes[2]
        self.input_rc[3] = joy_msg.axes[3]          # Throttle
        self.input_rc[4] = joy_msg.buttons[0]
        self.input_rc[5] = joy_msg.buttons[1]
        self.input_rc[6] = joy_msg.buttons[2]
        self.input_rc[7] = joy_msg.buttons[3]


if __name__ == '__main__':
    rospy.init_node('controller')
    try:
        controller_node = ControllerNode()
        controller_node.run()
    except rospy.ROSInterruptException: pass