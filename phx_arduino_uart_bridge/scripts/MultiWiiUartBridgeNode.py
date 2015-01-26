#!/usr/bin/env python
__author__ = 'manuelviermetz'

import numpy as np
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import FluidPressure #Barometer
from sensor_msgs.msg import Temperature #For compensation gyrodrift
from sensor_msgs.msg import Range #Distance to ground
from diagnostic_msgs.msg import DiagnosticArray #For Battery status
from phx_arduino_uart_bridge.msg import Motor

class MultiWiiUartBridgeNode():
    def __init__(self):
        print '>>>MultiWiiUartBridgeNode init'
        """
            this publishes:
            imu
            magnetometer // orientation could already be fused into the pose
            gps

            this receives:
            motor rpm messages

        """
        
        self.ImuPublisher = rospy.Publisher('/phoenix/imu', Imu, queue_size=10)
        self.NavSatFixPublisher = rospy.Publisher('/phoenix/gps', NavSatFix, queue_size=10)
        self.motorSubscriber = rospy.Subscriber('/phoenix/cmd_motor', Motor, self.motorMessageCallback)

        self.freq = 100 #hz
        r = rospy.Rate(self.freq)
        
        while not rospy.is_shutdown():
            

    def motorMessageCallback(self, imu_msg):
        print '>>>motorMessageCallback not implemented'

    def send_motors(self, motors=(1, 2, 3, 4), debug=False):
        """
         motors = [ motor0, motor1, motor2, motor3 ]
        """
        try:
            motor_msg = MotorMessage()
            motor_msg.motor0 = motors[0]
            motor_msg.motor1 = motors[1]
            motor_msg.motor2 = motors[2]
            motor_msg.motor3 = motors[3]
            self.ros_publish_motor.publish(motor_msg)
            if debug: print ' >>> sent rc2'
        except:
            print '>>> error in ros send_motor!'

    def send_rc0(self, rc0=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
         rc0 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        print ' >>> send_rc0 not implemented'

    def send_imu(self, imu=(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13), debug=False):
        """
         imu = [ accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY. magZ, pitch, roll, heading, altitude ]
        """
        print ' >>> send_imu not implemented'

    def send_battery(self, battery=(1, 2, 3, 0), debug=False):
        """
         battery = [ cell1, cell2, cell3, cell4 ]
        """
        print ' >>> send_battery not implemented'

    def receive_simple_directions(self, directions=(0, 0, 0, 0), debug=False):
        """
            this is a very basic approach to set flight directions!
            simple_directions = [0, 0, 0, 0]   # backward-forward, left-right, up-down, left-right-turn
        """
        foo = directions
        print ' >>> receive_simple_directions not implemented'


if __name__ == '__main__':
    
    rospy.init_node('MultiWiiUartBridge')
    try:
        multiWiiUartBridgeNode = MultiWiiUartBridgeNode()
    except rospy.ROSInterruptException:
        pass