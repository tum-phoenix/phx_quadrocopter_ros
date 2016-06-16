#!/usr/bin/env python
import numpy as np
import rospy
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import RemoteControl
from sensor_msgs.msg import Imu

#If we want our code to work properly we have to run another class: AltitudeHoldNode()
#It should be working before our device reaches the altitude of 1m


class Node():
    def __init__(self):
        rospy.init_node('landing_controller')
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.sub_imu = rospy.Subscriber('/phx/imu', Imu, self.imuCallback)
        self.sub = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, self.altitudeCallback)

        self.pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)
        self.enabled = False
        self.p = 1
        self.d = 5
        self.setPoint_d = 9.81
        self.setPoint = 1


        self.freq = 100  # Hz
        self.r = rospy.Rate(self.freq)


    def enableCallback(self, enable):
        self.enabled = enable

    def run(self):
        while not rospy.is_shutdown():
            if self.enabled:
                runSub()
            self.r.sleep()

    @abstractmethod
    def runsub(selfself):
        return
