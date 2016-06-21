#!/usr/bin/env python
import rospy
import numpy as np
from phx_uart_msp_bridge.msg import RemoteControl
from phx_uart_msp_bridge.msg import AutoPilotCmd
from phx_uart_msp_bridge.msg import ControllerCmd


class Autopilot:
    """
    The Autopilot fuses the incoming cmd_vel commands from different nodes like altitude hold node, gps position
    planner node and collision prevention to one outgoing command for the flight controller.

    the cmd_vel topic is used with
    node_identifier:
    nothing 0
    altitude_hold_node 1
    attitude_hold_node 2
    landing_node 3
    take_off_node 4
    map_position_node 5

    """
    def __init__(self):
        rospy.init_node('Autopilot_node')
        self.enabled_nodes = [False, False, False, False, False]
        self.enabled_nodes[0] = input('altitude on? ')

        self.enabled_nodes[1] = input('attitude on? ')
        self.enabled_nodes[2] = input('landing on? ')
        self.enabled_nodes[3] = input('starting on? ')
        self.enabled_nodes[4] = input('map_position on? ')

        self.autopilot_input_sub = rospy.Subscriber('/phx/autopilot/input', AutoPilotCmd, self.callback_input)
        self.rc_pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)
        self.controller_commands = rospy.Publisher('/phx/controller_commands', ControllerCmd, queue_size=1)

        self.current_pose = RemoteControl()
        self.rate = rospy.Rate(30)
    '''
    def callback_input(self, input_msg=AutoPilotCmd()):
        from_node = input_msg.node_identifier
        priority = input_msg.priority

        linear_x = input_msg.cmd.linear.x           # forward backward      [-1; 1]
        linear_y = input_msg.cmd.linear.y           # left right            [-1; 1]
        linear_z = input_msg.cmd.linear.z           # up down               [-1; 1]
        angular_z = input_msg.cmd.angular.z         # rotation heading      [-1; 1]

        if linear_x != 0:
            linear_x = np.clip(linear_x, -1, 1)
            self.current_pose.pitch = 1500 + linear_x*200

        if linear_y != 0:
            linear_y = np.clip(linear_y, -1, 1)
            self.current_pose.roll = 1500 + linear_y*200

        if linear_z != 0:
            linear_z = np.clip(linear_z, -1, 1)
            self.current_pose.throttle = 1500 + linear_z*500

        if angular_z != 0:
            angular_z = np.clip(angular_z, -0.2, 0.2)
            self.current_pose.yaw = 1500 + angular_z*500

        # publish new mixed remote control to flight controller
        rc_msg = RemoteControl()
        rc_msg.pitch = self.current_pose.pitch
        rc_msg.roll = self.current_pose.roll
        rc_msg.yaw = self.current_pose.yaw
        rc_msg.throttle = self.current_pose.throttle
        rc_msg.aux1 = 1000
        rc_msg.aux2 = 1900
        rc_msg.aux3 = 1900
        rc_msg.aux4 = int(1000 + np.random.random()*1000)

        self.rc_pub.publish(rc_msg)
    '''
    def run(self):
        while not rospy.is_shutdown():


            #if self.mode == 1:
            command = ControllerCmd()
            command.enabled =  self.enabled_nodes

            self.controller_commands.publish(command)

            print 'current pose \n', self.current_pose

            self.rate.sleep()


if __name__ == '__main__':
    try:
        autopilot_node = Autopilot()
        autopilot_node.run()
    except rospy.ROSInterruptException:
        pass
