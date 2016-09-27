#!/usr/bin/env python

import rospy

from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import AutoPilotCmd
import datetime

# WIP simulation use classes in folder future_controller in future
class SimulationMain():
    def __init__(self):
        print "hallo"
        rospy.init_node('simulation')
        self.altitude_topic = rospy.Publisher('/phx/marvicAltitude/altitude', Altitude, queue_size=1)
        self.command_topic = rospy.Subscriber('/phx/autopilot/input', AutoPilotCmd,  self.update_commands)
        # position
        self.x = 0.0 # forward
        self.y = 0.0 # left
        self.z = 0.0 # up
        # orientation (euler angle)
        self.a = 0.0
        self.b = 0.0
        self.c = 0.0

        # inputs (max: 1000, min: 2000)
        self.throttle = 1000
        self.pitch = 1000
        self.roll = 1000
        self.yaw = 1000

        # copter values
        self.weight = 2.5 # in kg
        self.motorForce = 40 # force of all motors in Newton

        self.g = 10 # m/s^2, earth acceleration
        self.maxAcc = self.motorForce / self.weight # m/s^2
        self.hoverAcc = self.g # m/s^2
        self.hovering = 0.60 # percentage of throttle for hovering
        self.minAcc = 0.0
        self.slope = 0.0
        self.frequency = 50 # Hz update frequency
        self.height = 0 # starting height
        self.r = rospy.Rate(self.frequency)
        print "ende"

    def update_commands(self, autopilot_msg):
        self.throttle = autopilot_msg.rc.throttle


    # minAcc is acceleration with throttle = 0
    def calculateMinAcc(self):
        self.slope = (self.maxAcc - self.hoverAcc) / (1 - self.hovering)
        self.minAcc = self.hoverAcc - (self.slope * self.hovering)



    # returns acceleration in m/s^2 for input
    def get_z_acceleration(self, throttle):
        percentage = (throttle - 1000.0) / 1000.0
        print percentage
        print "slope:" , self.slope, "minAcc:" , self.minAcc
        return self.minAcc + percentage * self.slope - self.g


    def calculate_height(self, throttle):
        dt = 1.0/self.frequency
        print "dt,", dt
        dH = 0.5 * self.get_z_acceleration(throttle) * dt * dt
        print "dh,", dH
        self.height += dH
        altitude_msg = Altitude()
        altitude_msg.estimated_altitude = self.height
        self.altitude_topic.publish(altitude_msg)

    def run(self):
        self.calculateMinAcc()
        print "run start"
        while not rospy.is_shutdown():
            print "loop"
            self.calculate_height(self.throttle)
            print self.height
            print "still alive"
            self.r.sleep()
            


if __name__ == '__main__':
    try:
        simulation = SimulationMain()
        simulation.run()
    except rospy.ROSInterruptException:
        print "exception"
        pass




