#!/usr/bin/env python
import numpy as np
import rospy
from pid_controller import PIDController
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import AutoPilotCmd
from phx_uart_msp_bridge.msg import RemoteControl
from phx_uart_msp_bridge.msg import ControllerCmd
from sensor_msgs.msg import Joy


class AltitudeHoldNode():
    def __init__(self):
        rospy.init_node('altitude_hold_controller')
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.node_identifier = 1
        self.sub = rospy.Subscriber('/phx/rc_marvic', Joy, self.rcCallback)
        self.sub = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, self.altitudeCallback)
        self.autopilot_commands = rospy.Subscriber('/phx/controller_commands', ControllerCmd, self.controllerCommandCallback)
#        self.rc_pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)
        self.altitude_pub = rospy.Publisher('/phx/autopilot/input', AutoPilotCmd, queue_size=1)

        setPoint_p = 1
        self.enabled = True
        p = 1
        i = 0
        d = 4
        setPoint_d = 0
        i_stop = 1
        controlCommand = 1500

        self.altitudeController = PIDController(controlCommand, p, i, d, setPoint_p, i_stop, setPoint_d, 0)

        self.freq = 100  # Hz
        self.r = rospy.Rate(self.freq)
        self.previousAltitude = 0
        self.firstCall = 1

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def controllerCommandCallback(self, controller_msg):
        self.enabled = controller_msg.enabled[self.node_identifier]


    def altitudeCallback(self, altitude_msg):
        if self.enabled:
            print("Altitude Callback")
            # set previousAltitude to current Altitude in first call
            if self.firstCall:
                self.previousAltitude = altitude_msg.estimated_altitude
                self.firstCall = 0

            un_cliped = self.altitudeController.calculate_control_command(altitude_msg.estimated_altitude, (altitude_msg.estimated_altitude - self.previousAltitude) * 100)

            if self.input_rc[4] > 1500:
                un_cliped = self.input_rc[3]

            controlCommand = np.clip(un_cliped, 1000, 2000)
            autopilot_command = AutoPilotCmd()
            autopilot_command.rc.throttle = controlCommand
            autopilot_command.node_identifier = self.node_identifier

            # Replay and override current rc

            self.previousAltitude = altitude_msg.estimated_altitude
    #        self.rc_pub.publish(joy_msg)
            self.altitude_pub.publish(autopilot_command)

            print  'alt:', altitude_msg.estimated_altitude, 'controlCommand:', controlCommand

        else:
            print 'altitude hold node disabled'

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
        controller_node.enabled = True
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
