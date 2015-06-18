#!/usr/bin/env python

import threading
import struct
import numpy as np

import rospy
from phx_arduino_uart_bridge.msg import Battery
from phx_arduino_uart_bridge.msg import LED


rospy.init_node('msg_publisher')
ros_publisher_battery = rospy.Publisher('/phoenix/stat_battery', Battery, queue_size=1)


while True == True:
    voltage = input(' >')
    try:
        voltage = int(voltage)
    except:
        voltage = 0
    print 'sending ', voltage
    cur_battery_stat = Battery()

    cur_battery_stat.cell1 = int(voltage)
    cur_battery_stat.cell2 = int(voltage)
    cur_battery_stat.cell3 = int(voltage)

    ros_publisher_battery.publish(cur_battery_stat)