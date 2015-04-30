#!/usr/bin/env python
__author__ = 'manuelviermetz'

import time
import pyCopter
from direct_serial_ros_bridge import RosCom, SerialCom


serial_multiwii = SerialCom(serial_port='/dev/multiwii', baudrate=115200)

ros_node = RosCom(node_name='multiwii_direct',
                  update_rate=500,
                  preset='MultiwiiSerial',
                  callback_option=serial_multiwii)

# dict of msg_codes: [frequency, time_of_next_request]
# if frequency is 0 this means no update at all.
# 66:   battery
# 101:  status                      2Hz
# 102:  imu                         20Hz
# 104:  motor                       25Hz
# 105:  rc          * interesting   50Hz
# 106:  gps         * interesting   15Hz
# 108:  attitude                    20Hz
# 109:  altitude    * interesting   15Hz
serial_multiwii.request_rates = {66:  [0, 0],
                                 101: [2, 0],
                                 102: [20, 0],
                                 104: [25, 0],
                                 105: [50, 0],
                                 106: [15, 0],
                                 108: [20, 0],
                                 109: [15, 0]}

print 'start done'

serial_communication = pyCopter.speedtest()
ros_communication = pyCopter.speedtest()

start_node_time = time.time()
while not ros_node.is_shutdown():
    serial_communication.start()
    serial_multiwii.request(debug=False)
    serial_communication.stop()

    ros_communication.start()
    ros_node.listen()
    ros_communication.stop()

    if serial_communication.print_result(rate=0.2, text='MultiWii serial communication:'):
        print 'MultiWii serial statistic:', int(time.time()-start_node_time), serial_multiwii.message_statistic
    ros_communication.print_result(rate=0.2, text='MultiWii ros communication:')


print 'shutting down direct_multiwii_ros_bridge'