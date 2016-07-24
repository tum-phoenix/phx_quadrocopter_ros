#!/usr/bin/env python
import numpy as np
import rospy
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import RemoteControl
from phx_uart_msp_bridge.msg import AutoPilotCmd
#from phx_uart_msp_bridge.msg import ControllerCmd
from sensor_msgs.msg import Imu
from PIDController import PIDController

#If we want our code to work properly we have to run another class: AltitudeHoldNode()
#It should be working before our device reaches the altitude of 1m


class LandingNode():
    def __init__(self):
        rospy.init_node('landing_controller')
        self.node_identifier = 3
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.sub_imu = rospy.Subscriber('/phx/imu', Imu, self.imuCallback)
#        self.autopilot_commands = rospy.Subscriber('/phx/controller_commands', ControllerCmd, self.controllerCommandCallback)
        self.sub = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, self.altitudeCallback)
        self.pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)

#        self.autopilot_commands = rospy.Subscriber('/phx/controller_commands', ControllerCmd, self.controllerCommandCallback)
        self.pub = rospy.Publisher('/phx/autopilot/input', AutoPilotCmd, queue_size=1)



        self.enabled = True
        self.p = 1
        self.d = 5
        self.setPoint_d = 9.81
        self.setPoint = 1


        self.freq = 100  # Hz
        self.r = rospy.Rate(self.freq)

        self.altitude_start = 1
        self.altitude = 0.0
        self.linear_acceleration_z = 0.0

        self.controlCommand = 1500

        self.landController = PIDController(1500, 1,0,5,1,0,9.81,0)


    def altitudeCallback(self, altitude_msg):
        self.altitude = altitude_msg.estimated_altitude

    def imuCallback(self, imu_msg):
        self.linear_acceleration_z = imu_msg.linear_acceleration.z

    def controllerCommandCallback(self, controller_msg):
        self.enabled = controller_msg.enabled[self.node_identifier]

    def run(self):
        while not rospy.is_shutdown():
            if self.enabled:
                #controlCommand_p = (self.setPoint - self.altitude) * self.p
                #controlCommand_d = (self.setPoint_d - self.linear_acceleration_z) * self.d
                un_cliped = self.landController.calculateControlCommand(self.altitude,self.linear_acceleration_z)
                #un_cliped = self.controlCommand + controlCommand_p + controlCommand_d
                controlCommand = np.clip(un_cliped, 1000, 2000)

                #print(self.controlCommand, self.altitude, self.linear_acceleration_z)
                autopilot_command = AutoPilotCmd()
                autopilot_command.rc.throttle = controlCommand
                autopilot_command.node_identifier = self.node_identifier
                print controlCommand
        self.r.sleep()
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
        controller_node.enabled = True
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
