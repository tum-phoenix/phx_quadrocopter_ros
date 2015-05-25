#!/usr/bin/env python
import time
import rospy
import numpy as np
from phx_arduino_uart_bridge.msg import Altitude
from phx_arduino_uart_bridge.msg import Autonomous
from phx_arduino_uart_bridge.msg import Battery
from phx_arduino_uart_bridge.msg import Status
from phx_arduino_uart_bridge.msg import Motor
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from phx_gui.msg import GUI_cmd


class testNode():
    def __init__(self):
        self.sub_gui = rospy.Subscriber('/phx/gui_parameters', GUI_cmd, self.GUI_cmd_Callback)
        self.sub_gui = rospy.Subscriber('/phx/motor_marvic', Motor, self.Motor_Callback)
        self.pub_motor_computer = rospy.Publisher('/phx/motor_computer', Motor, queue_size=1)
        self.pub_rc_computer = rospy.Publisher('/phx/rc_computer', Joy, queue_size=1)

        self.freq = 100     # Hz
        self.r = rospy.Rate(self.freq)

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def Altitude_Callback(self, altitude_msg):
        estimated_altitude = altitude_msg.estimated_altitude
        variation_speed = altitude_msg.variation

    def Motor_Callback(self, motor_msg):
        print 'motors:', motor_msg.motor0, motor_msg.motor1, motor_msg.motor2, motor_msg.motor3

    def rcCallback(self, joy_msg):
        input_rc = [0, 0, 0, 0, 0, 0, 0, 0]
        input_rc[0] = joy_msg.axes[0]
        input_rc[1] = joy_msg.axes[1]
        input_rc[2] = joy_msg.axes[2]
        input_rc[3] = joy_msg.axes[3]          # Throttle
        input_rc[4] = joy_msg.buttons[0]
        input_rc[5] = joy_msg.buttons[1]
        input_rc[6] = joy_msg.buttons[2]
        input_rc[7] = joy_msg.buttons[3]

    def GUI_cmd_Callback(self, GUI_cmd_msg):
        parameter0 = 1.0 * int(GUI_cmd_msg.gui_cmd_0) / 100
        parameter1 = 1.0 * int(GUI_cmd_msg.gui_cmd_1) / 100
        parameter2 = 1.0 * int(GUI_cmd_msg.gui_cmd_2) / 100
        parameter3 = 1.0 * int(GUI_cmd_msg.gui_cmd_3) / 100
        parameter4 = 1.0 * int(GUI_cmd_msg.gui_cmd_4) / 100
        parameter5 = 1.0 * int(GUI_cmd_msg.gui_cmd_5) / 100
        parameter6 = 1.0 * int(GUI_cmd_msg.gui_cmd_6) / 100
        parameter7 = 1.0 * int(GUI_cmd_msg.gui_cmd_7) / 100
        self.publish_motor_computer(1000 + 1000 * np.array([parameter0, parameter1, parameter2, parameter3, parameter4, parameter5, parameter6, parameter7]))

    def publish_rc_computer(self, rc_command):
        clipped_rc_command = np.clip(rc_command, 1000, 2000)
        joy_msg = Joy()
        joy_msg.axes = clipped_rc_command[0:4]
        joy_msg.buttons = clipped_rc_command[4:]
        self.pub_rc_computer.publish(joy_msg)

    def publish_motor_computer(self, motor_command):
        clipped_motor_command = np.clip(motor_command, 1000, 2000)
        motor_msg = Motor()
        motor_msg.motor0 = clipped_motor_command[0]
        motor_msg.motor1 = clipped_motor_command[1]
        motor_msg.motor2 = clipped_motor_command[2]
        motor_msg.motor3 = clipped_motor_command[3]
        self.pub_motor_computer.publish(motor_msg)

if __name__ == '__main__':
    rospy.init_node('controller')
    try:
        test_Node = testNode()
        test_Node.run()
    except rospy.ROSInterruptException: pass