__author__ = 'manuelviermetz'

from direct_serial_ros_bridge import RosCom, SerialCom


serial_multiwii = SerialCom(serial_port='/dev/multiwii', baudrate=115200)

ros_node = RosCom(node_name='multiwii_direct',
                  update_rate=50,
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
                                 101: [1, 0],
                                 102: [30, 0],
                                 104: [9, 0],
                                 105: [10, 0],
                                 106: [11, 0],
                                 108: [31, 0],
                                 109: [2, 0]}

print 'start done'

while True:
    serial_multiwii.request()
    serial_multiwii.receive()
    ros_node.listen()