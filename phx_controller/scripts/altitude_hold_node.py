#!/usr/bin/env python
import numpy as np
import rospy
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import RemoteControl
from sensor_msgs.msg import Joy


class AltitudeHoldNode():
    def __init__(self):
        rospy.init_node('altitude_hold_controller')
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.sub = rospy.Subscriber('/phx/rc_marvic', Joy, self.rcCallback)
        self.sub = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, self.altitudeCallback)
#        self.rc_pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)
        self.altitude_pub = rospy.Publisher('/phx/fc/altitude_hold', RemoteControl, queue_size=1)

        self.setPoint = 1

        self.p = 1
        self.d = 4
        self.setPoint_d = 0
        self.i = 0
        self.sum_i = 0
        self.i_stop = 1
        self.controlCommand = 1500

        self.freq = 100  # Hz
        self.r = rospy.Rate(self.freq)
        self.previousAltitude = 0
        self.firstCall = 1

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def altitudeCallback(self, altitude_msg):
        print("Altitude Callback")
        # set previousAltitude to current Altitude in first call
        if self.firstCall:
            self.previousAltitude = altitude_msg.estimated_altitude
            self.firstCall = 0

        if self.input_rc[4] > 1500:
            self.setPoint = altitude_msg.estimated_altitude
            self.controlCommand = self.input_rc[3]

        self.sum_i += self.setPoint - altitude_msg.estimated_altitude
        if self.sum_i >= self.i_stop:
            self.sum_i = self.i_stop
        elif self.sum_i <= -self.i_stop:
            self.sum_i = -self.i_stop

        controlCommand_p = (self.setPoint - altitude_msg.estimated_altitude) * self.p
        controlCommand_d = (self.setPoint_d - (altitude_msg.estimated_altitude - self.previousAltitude) * 100) * self.d
        controlCommand_i = self.sum_i * self.i
        un_cliped = self.controlCommand + controlCommand_p + controlCommand_d + controlCommand_i
        self.controlCommand = np.clip(un_cliped, 1000, 2000)
        joy_msg = RemoteControl()

        # Replay and override current rc
        joy_msg.pitch = self.input_rc[0]
        joy_msg.roll = self.input_rc[1]
        joy_msg.yaw = self.input_rc[2]
        joy_msg.throttle = self.controlCommand
        joy_msg.aux1 = self.input_rc[4]
        joy_msg.aux2 = self.input_rc[5]
        joy_msg.aux3 = self.input_rc[6]
        joy_msg.aux4 = self.input_rc[7]

        self.previousAltitude = altitude_msg.estimated_altitude
#        self.rc_pub.publish(joy_msg)
        self.altitude_pub.publish(joy_msg)

        print 'set_point:', self.setPoint, '\t alt:', altitude_msg.estimated_altitude
        print 'controlCommand', un_cliped, self.controlCommand, 'p:', controlCommand_p, 'i:', controlCommand_i

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
    try:
        controller_node = AltitudeHoldNode()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass