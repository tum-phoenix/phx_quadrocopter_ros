__author__ = 'manuelviermetz'

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


from OSC_com import *

# init osc ----------------------------------------------------------------------------------------------------------------------------------------------------
osc_transmitter = OSCt(port=10001)
osc_receiver = OSCr(osc_transmitter, port=10000)
osc_receiver.start()


# init ros - osc bridge ---------------------------------------------------------------------------------------------------------------------------------------
def default_ros_subscribe_callback(message, ros_topic):
    # ros  >> osc  >> gui
    msg_payload = []
    if type(message) == Altitude:
        msg_payload.append(message.estimated_altitude)
        msg_payload.append(message.variation)
    elif type(message) == Autonomous:
        msg_payload.append(message.is_autonomous)
    elif type(message) == Battery:
        msg_payload.append(message.cell1)
        msg_payload.append(message.cell2)
        msg_payload.append(message.cell3)
        msg_payload.append(message.cell4)
    elif type(message) == Status:
        msg_payload.append(message.cycleTime)
        msg_payload.append(message.i2c_errors_count)
    else:
        print '>>> default_ros_callback: not implemented topic:', ros_topic
    if len(msg_payload) > 0:
        osc_transmitter.send_topic(topic=ros_topic, payload=msg_payload)


def default_ros_publish_callback(add, tag, stuff, source):
    # ros  << osc  << gui
    # input looks like incoming OSC: /gyrosc/gyro fff [0.48758959770202637, 0.06476165354251862, -0.19856473803520203] ('192.168.0.33', 57527)
    if add in publishers.keys():
        publisher = publishers[add]
        if publisher[1] == 'Joy':
            msg = Joy()
            msg.axes = stuff[0:4]
            msg.buttons = stuff[4:]
        elif publisher[1] == 'Motor':
            msg = Motor()
            msg.motor0 = stuff[0]
            msg.motor1 = stuff[1]
            msg.motor2 = stuff[2]
            msg.motor3 = stuff[3]
        elif publisher[1] == 'GUI_cmd':
            msg = GUI_cmd()
            msg.gui_cmd_0 = stuff[0]
            msg.gui_cmd_1 = stuff[1]
            msg.gui_cmd_2 = stuff[2]
            msg.gui_cmd_3 = stuff[3]
            msg.gui_cmd_4 = stuff[4]
            msg.gui_cmd_5 = stuff[5]
            msg.gui_cmd_6 = stuff[6]
            msg.gui_cmd_7 = stuff[7]
        else:
            print '>>> default_ros_publish_callback: not implemented msg_type:', publisher[1]
            return
        publisher[0].publish(msg)
    else:
        print '>>> default_ros_publish_callback: not implemented topic:', add


# init ros ----------------------------------------------------------------------------------------------------------------------------------------------------
rospy.init_node('gui')
freq = 5
r = rospy.Rate(freq)


# build ros - osc bridge --------------------------------------------------------------------------------------------------------------------------------------
# ros       >> osc  >> gui
# subscribe >> send >> receive
subscribers = {'/phx/altitude_marvic': rospy.Subscriber('/phx/altitude_marvic', Altitude, lambda msg: default_ros_subscribe_callback(msg, '/phx/altitude_marvic')),
               '/phx/status_marvic': rospy.Subscriber('/phx/status_marvic', Status, lambda msg: default_ros_subscribe_callback(msg, '/phx/status_marvic')),
               '/phx/battery_marvic': rospy.Subscriber('/phx/battery_marvic', Battery, lambda msg: default_ros_subscribe_callback(msg, '/phx/battery_marvic'))
               }

# ros     << osc     << gui
# publish << receive << send
publishers = {'/phx/gui_rc': [rospy.Publisher('/phx/gui_rc', Joy, queue_size=1), 'Joy'],
              '/phx/gui_parameters': [rospy.Publisher('/phx/gui_parameters', GUI_cmd, queue_size=1), 'GUI_cmd']
              }
for receive_topic in publishers.keys():
    osc_receiver.add_receive_message(receive_topic, default_ros_publish_callback)

# main loop ---------------------------------------------------------------------------------------------------------------------------------------------------
counter = 0
try:
    while not rospy.is_shutdown():

        print 'osc connection status: OSCr', osc_receiver.connection_status, 'OSCt', osc_transmitter.connection_status, osc_transmitter.sending_failed_counter, time.time() - osc_receiver.time_of_last_keep_alive_receiving
        osc_receiver.keep_alive()
        counter += 1
        print 'sent... ', counter

        r.sleep()
    osc_receiver.stop()
    osc_transmitter.stop()
except:
    osc_receiver.stop()
    osc_transmitter.stop()