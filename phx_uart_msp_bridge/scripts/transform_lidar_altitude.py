#!/usr/bin/env python

import numpy as np
import rospy
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import Attitude

pitch_angle = 0    # rad
roll_angle = 0     # rad

def convert_altitude_measurement(input_altitude):
    global pitch_angle, roll_angle
    new_altitude = 1.0 * input_altitude.estimated_altitude * np.cos(pitch_angle/180*np.pi) * np.cos(roll_angle/180*np.pi)
    new_msg = Altitude()
    new_msg.estimated_altitude = new_altitude
    ros_publish_new_altitude.publish(new_msg)

def receive_attitude(input_attitude):
    global pitch_angle, roll_angle
    pitch_angle = input_attitude.pitch
    roll_angle = input_attitude.roll

rospy.init_node('transform_lidar_altitude')
ros_subscribe_attitude = rospy.Subscriber('/phx/fc/attitude', Attitude, receive_attitude)
ros_subscribe_altitude = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, convert_altitude_measurement)
ros_publish_new_altitude = rospy.Publisher('/phx/altitude', Altitude, queue_size=1)

r = rospy.Rate(10)

while not rospy.is_shutdown():
    r.sleep()
