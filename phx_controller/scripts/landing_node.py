#!/usr/bin/env python
import numpy as np
import rospy
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import RemoteControl
from sensor_msgs.msg import Imu


class LandingNode():
    def __init__(self):
        rospy.init_node('landing_controller')
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.sub_imu = rospy.Subscriber('/phx/imu', Imu, self.imuCallback)
        self.sub = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, self.altitudeCallback)

        self.pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)

        self.p = 1
        self.d = 5
        self.setPoint_d = 0
        self.setPoint = 0

        self.controlCommand = 1500

        self.freq = 100  # Hz
        self.r = rospy.Rate(self.freq)

        self.altitude_start = 1
        self.altitude = 0.0
        self.linear_acceleration_z = 0.0

        self.controlCommand = 1500
        self.asoll

    def altitudeCallback(self, altitude_msg):
        self.altitude = altitude_msg.estimated_altitude

    def imuCallback(self, imu_msg):
        self.linear_acceleration_z = imu_msg.linear_acceleration.z

    def run(self):
        while not rospy.is_shutdown():
            controlCommand_p = (self.setPoint - self.altitude) * self.p
            controlCommand_d = (self.setPoint_d - self.linear_acceleration_z) * self.d

            un_cliped = self.controlCommand + controlCommand_p + controlCommand_d
            self.controlCommand = np.clip(un_cliped, 1000, 2000)

        '''
        if(self.altitude < self.altitude_start):
                if(self.linear_acceleration_z > 0.1):
                    self.throttle += 2
            else:
                self.throttle -= 10
        '''



if __name__ == '__main__':
    try:
        controller_node = LandingNode()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
