#!/usr/bin/env python

import numpy as np
import rospy

from sensor_msgs.msg import JointState
from phx_uart_msp_bridge.msg import Servo


def receive_states(current_states):
    print 'received new states'
    global joint_positions
    joint_positions = current_states.position


rospy.init_node('joint_states_to_Servo_bridge')
ros_subscribe_joint_states = rospy.Subscriber('/joint_states', JointState, receive_states)
ros_publish_uart_servos = rospy.Publisher('/servo/uart_servo_cmd', Servo, queue_size=1)

servo_msg = Servo()
joint_positions = [0.0] * 18

r = rospy.Rate(50)


while not rospy.is_shutdown():
    print joint_positions
    servo_msg.header.stamp = rospy.Time.now()
    servo_msg.servo6 = 1500 + 2000. * joint_positions[0] / np.pi    # r1
    servo_msg.servo7 = 1500 - 2000. * joint_positions[1] / np.pi    # r1
    servo_msg.servo8 = 1500 + 2000. * joint_positions[2] / np.pi    # r1
    servo_msg.servo2 = 1500 + 2000. * joint_positions[3] / np.pi    # r2
    servo_msg.servo4 = 1500 - 2000. * joint_positions[4] / np.pi    # r2
    servo_msg.servo5 = 1500 + 2000. * joint_positions[5] / np.pi    # r2
    servo_msg.servo3 = 1500 + 2000. * joint_positions[6] / np.pi    # r3
    servo_msg.servo0 = 1500 - 2000. * joint_positions[7] / np.pi    # r3
    servo_msg.servo1 = 1500 + 2000. * joint_positions[8] / np.pi    # r3
    servo_msg.servo11 = 1500 + 2000. * joint_positions[9] / np.pi    # l1
    servo_msg.servo10 = 1500 + 2000. * joint_positions[10] / np.pi  # l1
    servo_msg.servo9 = 1500 - 2000. * joint_positions[11] / np.pi  # l1
    servo_msg.servo14 = 1500 + 2000. * joint_positions[12] / np.pi  # l2
    servo_msg.servo13 = 1500 + 2000. * joint_positions[13] / np.pi  # l2
    servo_msg.servo12 = 1500 - 2000. * joint_positions[14] / np.pi  # l2
    servo_msg.servo16 = 1500 + 2000. * joint_positions[15] / np.pi  # l3
    servo_msg.servo17 = 1500 + 2000. * joint_positions[16] / np.pi  # l3
    servo_msg.servo15 = 1500 - 2000. * joint_positions[17] / np.pi  # l3
    ros_publish_uart_servos.publish(servo_msg)
    r.sleep()
