#!/usr/bin/env python
import time
import rospy
import numpy as np
from phx_uart_bridge.msg import MotorMessage
from sensor_msgs.msg import Imu
from drone import Drone
from controller import Controller

class ControllerNode():
    def __init__(self):
        
        self.freq = 100 #hz
        #self.s = 3 #battery configuration 3s/4s
        #self.v_c = 4.7 # voltage per lipo cell
        #self.v_s = self.s*self.v_c # supply voltage
        #self.k_v = 580 # RPM/Volt e.g. MN3508 580/ MN3110 700Kv
        #self.idle_rpm = self.k_v*self.v_s #idle RPM
        self.prop_size=13 #in inch
        
        self.zero_rpm = 5974.71-350.00*self.prop_size
        self.hover_rpm = 5974.71-350.00*self.prop_size+5340*0.5 #half throttle RPM 4000
        self.full_rpm = 5974.71-350.00*self.prop_size+5340*1 #full throttle RPM 6600
        
        self.zero_rpm = 2500
        self.hover_rpm = 2600
        self.full_rpm = 2800
        
        print "controlling rpm between",self.zero_rpm," and ",self.full_rpm

        self.sub = rospy.Subscriber('/phoenix/imu', Imu, self.imuCallback)
        self.pub = rospy.Publisher('/phoenix/cmd_motor', MotorMessage)
        r = rospy.Rate(self.freq)
        
        self.drone = Drone()
        self.controller = Controller(self.drone)

        self.xdot_desired = np.zeros((3, 1))
        self.thetadot_desired = np.zeros((3, 1))
        
        self.last = rospy.get_time()
        
        while not rospy.is_shutdown():
            r.sleep()

    def imuCallback(self, imu_msg):
        now = rospy.get_time()
        dt = now - self.last
        self.last = now
        #update state
        self.drone.thetadot[0] = imu_msg.angular_velocity.x
        self.drone.thetadot[1] = imu_msg.angular_velocity.y
        self.drone.thetadot[2] = imu_msg.angular_velocity.z
        
        self.drone.xdoubledot[0] = imu_msg.linear_acceleration.x
        self.drone.xdoubledot[1] = imu_msg.linear_acceleration.y
        self.drone.xdoubledot[2] = imu_msg.linear_acceleration.z

        self.drone.xdot = self.drone.xdot#+self.drone.xdoubledot*dt
        
        #calculate motor commands
        cmds = self.controller.calculate_control_command(self.xdot_desired, self.thetadot_desired.item(2))
        print "cmds",cmds
        cmds = np.clip(cmds, 0, 3)
        print "clipped cmds",cmds

        cmds = self.zero_rpm +cmds*(self.full_rpm-self.hover_rpm)
        cmds = np.clip(cmds, 2500, 2800)
        

        #publish to topic/uart bridge
        motor_msg = MotorMessage()
        motor_msg.motor0 = cmds[0] #VL
        motor_msg.motor1 = cmds[1] #VR
        motor_msg.motor2 = cmds[2] #HL
        motor_msg.motor3 = cmds[3] #HR
        self.pub.publish(motor_msg)

        #test
        #for rpm in range(2400, 2900, +1):
        #   motor_msg.motor0 = rpm
        #   motor_msg.motor1 = rpm
        #   motor_msg.motor2 = rpm
        #   motor_msg.motor3 = rpm
        #   self.pub.publish(motor_msg)

if __name__ == '__main__':
    rospy.init_node('controller')
    try:
        controller_node = ControllerNode()
    except rospy.ROSInterruptException: pass