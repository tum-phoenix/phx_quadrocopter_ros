#!/usr/bin/env python
import time
__author__ = 'manuelviermetz'

from direct_serial_ros_bridge import RosCom, SerialCom


#serial_multiwii = SerialCom(serial_port='/dev/multiwii', baudrate=115200)
serial_multiwii = SerialCom(serial_port='/dev/marvic', baudrate=2000000)

ros_node = RosCom(node_name='multiwii_direct',
                  update_rate=100,
                  preset='MultiwiiSerial',
                  callback_option=serial_multiwii)

# dict of msg_codes: [frequency, time_of_next_request]
# if frequency is 0 this means no update at all.
# 66:   battery
# 101:  status
# 102:  imu
# 104:  motor
# 105:  rc
# 106:  gps
# 108:  attitude
# 109:  altitude
serial_multiwii.request_rates = {66:  [0, 0],
                                 101: [2, 0],
                                 102: [20, 0],
                                 104: [40, 0],
                                 105: [40, 0],
                                 106: [15, 0],
                                 108: [40, 0],
                                 109: [15, 0]}

print 'start done'

while not ros_node.is_shutdown():
    t0 = time.time()
    serial_multiwii.request()
    t1 = time.time()
    serial_multiwii.receive()
    t2 = time.time()
    ros_node.listen()
    print 'loop took', t1 - t0, t2 - t1, time.time() - t2

print 'shutting down direct_multiwii_ros_bridge'