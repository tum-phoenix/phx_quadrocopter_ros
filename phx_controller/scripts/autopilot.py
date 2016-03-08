#!/usr/bin/env python
import rospy
import numpy as np
#from sensor_msgs.msg import Joy
from phx_uart_msp_bridge.msg import RemoteControl
from geometry_msgs.msg import Twist

class Autopilot:
    def __init__(self):

        rospy.init_node('Autopilot_node')
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.callback_twist)
        self.rc_sub = rospy.Subscriber('/phx/rc_marvic', RemoteControl, self.callback_rc)
        self.rc_pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)


        self.current_pose = RemoteControl()
        self.rate = rospy.Rate(20)

    def callback_twist(self, twist_msg=Twist()):

        linear_x = twist_msg.linear.x
        linear_y = twist_msg.linear.y
        angular_z = twist_msg.angular.z

        linear_x = np.clip(linear_x,-1,1)
        self.current_pose.pitch= 1500 + linear_x*200
        linear_y = np.clip(linear_y,-1,1)
        self.current_pose.roll= 1500 + linear_y*200
        angular_z = np.clip(angular_z,-0.2,0.2)
        self.current_pose.yaw = 1500 + angular_z*500
        self.current_pose.throttle = 1500

        rc_msg=RemoteControl()
        rc_msg.pitch = self.current_pose.pitch
        rc_msg.roll = self.current_pose.roll
        rc_msg.yaw = self.current_pose.yaw
        rc_msg.throttle = self.current_pose.throttle
        rc_msg.aux1=1000
        rc_msg.aux2=1900
        rc_msg.aux3=1900
        rc_msg.aux4= int(1000 + np.random.random()*1000)

        self.rc_pub.publish(rc_msg)


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            print 'current pose \n', self.current_pose


if __name__ == '__main__':
    try:
        autopilot_node = Autopilot()
        autopilot_node.run()
    except rospy.ROSInterruptException:
        pass