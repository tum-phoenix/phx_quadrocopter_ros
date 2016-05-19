#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Imu, Joy
from phx_uart_msp_bridge.msg import RemoteControl
from tf import transformations


class GPSHoldNode():
    def __init__(self):
        rospy.init_node('map_position_hold')
        self.r = rospy.Rate(100)

        self.currentState = Imu()
        #current position in the map frame
        self.map_position = Point()
        self.target = Point(1, 1, 0)

        self.controlCommand_pitch = 1500
        self.controlCommand_roll = 1500
        self.controlCommand_yaw = 1500

        self.controlCommand_throttle = 1500

        #####PID parameters#####
        #yaw
        self.i_stop_yaw = 100
        self.i_sum_yaw = 0
        self.set_p_yaw = 0
        self.set_d_yaw = 0
        self.p_yaw = 1
        self.d_yaw = 1
        self.i_yaw = 1


        #position
        self.i_stop_position = 100
        self.set_i_position = 0
        self.set_d_position = 0
        self.i_position = 1
        self.p_position = 5
        self.d_position = 1
        self.i_sum_position_x = 1
        self.i_sum_position_y = 1

        self.rc_input = RemoteControl()
        self.rc_input.pitch = 1500
        self.rc_input.roll = 1500
        self.rc_input.yaw = 1500
        self.rc_input.throttle = 1500
        self.rc_input.aux1 = 1000
        self.rc_input.aux2 = 1000
        self.rc_input.aux3 = 1000
        self.rc_input.aux4 = 1000


        self.altitude_sub = rospy.Subscriber('/phx/fc/altitude_hold', RemoteControl, self.altitude_callback)
        self.rc_sub = rospy.Subscriber('/phx/rc_marvic', Joy, self.rc_callback)
        self.imu_sub = rospy.Subscriber('/phx/imu', Imu, self.imu_callback)
        self.pose_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, self.pose_callback)
        self.cmd_pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)

    def pose_callback(self, pose_msg):

        #get yaw from pose_msg::orientation quaternion
        pose = pose_msg.pose.orientation
        yaw = (transformations.euler_from_quaternion([pose.x, pose.y, pose.z, pose.w])[2])*(180/np.pi)
        print '\n-----------------------------------------------------------------\ncurrent yaw', yaw

        #correct the yaw to 0 to decouple pitch & roll
        self.i_sum_yaw += self.set_p_yaw - yaw
        if abs(self.i_sum_yaw) >= self.i_stop_yaw:
            self.i_sum_yaw = self.i_stop_yaw

        control_i_yaw = self.i_sum_yaw * self.i_yaw
        control_p_yaw = (self.set_p_yaw - yaw) * self.p_yaw
        control_d_yaw = (self.set_d_yaw - self.currentState.angular_velocity.z) * self.d_yaw
        unclipped = 1500 + control_d_yaw + control_p_yaw + control_i_yaw
        self.controlCommand_yaw = np.clip(unclipped, 1000, 2000)
        print 'control_yaw', self.controlCommand_yaw, '\tcontrol_p', control_p_yaw, '\tcontrol_d', control_d_yaw, '\tcontrol_i', control_i_yaw

        #proceed only if yaw is close to zero; correct x & y error at the same time
        if abs(yaw) <= 10:
            print 'yaw < 10; current position:\n', self.map_position

            self.map_position = pose_msg.pose.position

            #error in x direction (pitch)
            self.i_sum_position_x += self.target.x - self.map_position.y
            if abs(self.i_sum_position_x) >= self.i_stop_position:
                self.i_sum_position_x = self.i_stop_position

            control_i_position_x = self.i_sum_position_x * self.i_position
            control_p_position_x = (self.target.x - self.map_position.y) * self.p_position
            control_d_position_x = (self.set_d_position - self.currentState.angular_velocity.y) * self.d_position
            unclipped = 1500 + control_p_position_x + control_d_position_x
            self.controlCommand_pitch = np.clip(unclipped, 1000, 2000)
            print 'control_pitch', self.controlCommand_pitch, '\tcontrol_p', control_p_position_x, '\tcontrol_d', control_d_position_x, '\tcontrol_i', control_i_position_x

            #error in y direction (roll)
            self.i_sum_position_y += self.target.y - self.map_position.x
            if abs(self.i_sum_position_y) >= self.i_stop_position:
                self.i_sum_position_y = self.i_stop_position

            control_i_position_y = self.i_sum_position_y * self.i_position
            control_p_position_y = (self.target.y - self.map_position.x) * self.p_position
            control_d_position_y = (self.set_d_position - self.currentState.angular_velocity.x) * self.d_position
            unclipped = 1500 + control_p_position_y + control_d_position_y + control_i_position_y
            self.controlCommand_roll = np.clip(unclipped, 1000, 2000)
            print 'control_roll', self.controlCommand_roll, '\tcontrol_p', control_p_position_y, '\tcontrol_d', control_d_position_y, '\tcontrol_i', control_i_position_y


        #override current rc
        rc_message = self.rc_input
        rc_message.pitch = self.controlCommand_pitch
        rc_message.roll = self.controlCommand_roll
        rc_message.yaw = self.controlCommand_yaw
        rc_message.throttle = self.controlCommand_throttle
        #rc_message.throttle =
        self.cmd_pub.publish(rc_message)

    def altitude_callback(self, altitude_msg):
        self.controlCommand_throttle = altitude_msg.throttle

    def imu_callback(self,imu_msg):
        self.currentState = imu_msg

    def rc_callback(self,rc_msg):
        self.rc_input[0] = rc_msg.axes[0]
        self.rc_input[1] = rc_msg.axes[1]
        self.rc_input[2] = rc_msg.axes[2]
        self.rc_input[3] = rc_msg.axes[3]
        self.rc_input[4] = rc_msg.buttons[0]
        self.rc_input[5] = rc_msg.buttons[1]
        self.rc_input[6] = rc_msg.buttons[2]
        self.rc_input[7] = rc_msg.buttons[3]



    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()


if __name__ == '__main__':
    try:
        controller_node = GPSHoldNode()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
