__author__ = 'manuelviermetz'
# based on https://github.com/tum-phoenix/phx_controller/blob/master/src/controller_node.py

import time
import numpy as np

import rospy
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure #Barometer
from sensor_msgs.msg import Temperature #For compensation gyrodrift
from sensor_msgs.msg import Range #Distance to ground
from geometry_msgs.msg import Twist, Quaternion
from phx_arduino_uart_bridge.msg import Motor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue #For Battery status


class ros_osc_bridge(osc_transmitter=None):
    def __init__(self):
        if osc_transmitter:
            self.osc_transmitter = osc_transmitter
        else:
            self.osc_transmitter = None
        try:
            rospy.init_node('OSC_Bridge')
            self.ros_publish_cmd_rc1 = rospy.Publisher('/phoenix/cmd_rc1', Joy, queue_size=10)
            self.Joy_1_cmd_msg = Joy()
            self.ros_publish_cmd_vel = rospy.Publisher('/phoenix/cmd_vel', Joy, queue_size=10)
            self.cmd_vel_msg = Twist()

            self.ros_subscribe_stat_imu = rospy.Subscriber('/phoenix/stat_imu', Imu, self.callback_stat_imu)
            self.ros_subscribe_stat_motor = rospy.Subscriber('/phoenix/stat_motor', Motor, self.callback_stat_motor)
            self.ros_subscribe_stat_gps = rospy.Subscriber('/phoenix/stat_gps', NavSatFix, self.callback_stat_gps)
            self.ros_subscribe_stat_battery = rospy.Subscriber('/phoenix/stat_battery', DiagnosticArray, self.callback_stat_battery)
            self.ros_subscribe_stat_rc0 = rospy.Subscriber('/phoenix/stat_rc0', Joy, self.callback_stat_rc0)
            self.ros_subscribe_stat_rc1 = rospy.Subscriber('/phoenix/stat_rc1', Joy, self.callback_stat_rc1)
            self.ros_subscribe_stat_rc2 = rospy.Subscriber('/phoenix/stat_rc2', Joy, self.callback_stat_rc2)

            self.freq = 50     # Hz
            self.rate = rospy.Rate(self.freq)
        except:
            print ' >>> error in ros_osc_bridge __init__'

    def callback_stat_imu(self, stuff):
        print ' > not implemented jet, imu', stuff

    def callback_stat_motor(self, stuff):
        print ' > not implemented jet, motor', stuff

    def callback_stat_gps(self, stuff):
        print ' > not implemented jet, gps', stuff

    def callback_stat_battery(self, stuff):
        print ' > not implemented jet, battery', stuff

    def callback_stat_rc0(self, stuff):
        print ' > not implemented jet, rc0', stuff
        if self.osc_transmitter:
            self.osc_transmitter.send_rc0(stuff.axes + stuff.buttons)
            print '>>> osc out rc0', stuff

    def callback_stat_rc1(self, stuff):
        print ' > not implemented jet, rc1', stuff
        if self.osc_transmitter:
            self.osc_transmitter.send_rc1(stuff.axes + stuff.buttons)
            print '>>> osc out rc1', stuff

    def callback_stat_rc2(self, stuff):
        print ' > not implemented jet, rc2', stuff
        if self.osc_transmitter:
            self.osc_transmitter.send_rc2(stuff.axes + stuff.buttons)
            print '>>> osc out rc2', stuff

    def listen(self):
        """
            this makes rospy look for incoming messages which will be received by their callback functions.
        """
        self.rate.sleep()

    def callback_cmd_motor(self, stuff):
        print ' >>> ROS_callback: received cmd_motor', stuff
        self.copter.send_serial_motor(motor_values=stuff)

    def callback_cmd_vel(self, stuff):
        print ' >>> ROS_callback: received cmd_vel', stuff
        self.simple_directions = stuff
        self.calc_rc_from_simple_directions()
        print '     >>> not implemented jet'

    def callback_cmd_rc_1(self, stuff):
        """
            in case stuff = [ self.throttle, self.pitch, self.roll, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4 ]
        """
        try:
            self.set_sticks(sticks=stuff)
        except:
            print ' >>> ROS_callback: receive callback_cmd_rc_1 failed', stuff

    def pub_rc1(self, rc1=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
            rc1 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        try:
            self.Joy_1_msg.axes = rc1[:4]
            self.Joy_1_msg.buttons = rc1[4:]
            self.ros_publish_rc1.publish(self.Joy_1_msg)
            if debug: print ' >>> sent rc1'
        except:
            print '>>> error in ros pub_rc1!'



if __name__ == '__main__':
    try:
        ros_network = ros_communication()

        while True:
            # do some stuff like updating data via serial connection
            time.sleep(0.5)

            ros_network.pub_motors([2, 4, 6, 8])

    except rospy.ROSInterruptException:
        pass
