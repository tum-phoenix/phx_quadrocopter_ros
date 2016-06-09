#!usr/bin/env python
import numpy as np
import rospy
import time
from altitude_hold_node import AltitudeHoldNode
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import RemoteControl
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu

class TakeOffNode():
	def __init__(self):
		rospy.init_node('take_off_controller')

		self.sub_alt = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, self.altCallback)
	        self.sub_imu = rospy.Subscriber('/phx/imu', Imu, self.imuCallback)

		self.p = 5	#PID controller
		self.d = 1
		self.i = 0.01

		self.i_sum = 0
		self.i_stop = 100

		self.setPoint_p = 1	#wanted altitude
		self.setPoint_d = 10	#wanted acceleration

		self.altitude = 0
		self.linear_acceleration_z = 0

		self.controlCommand = 1300	#throttle
		self.previousAltitude = 0

		self.freq = 100
		self.r = rospy.Rate(self.freq)

#Following two functions read altitude and acceleration from the sensors

	def altCallback(self, alt_msg):
		#self.altitude = alt_msg.estimated_altitude
		self.altitude = 0.2

	def imuCallback(self, imu_msg):
		#self.linear_acceleration_z = imu_msg.linear_acceleration.z
		self.linear_acceleration_z = 8

	def run(self):
		while not rospy.is_shutdown():

			if(self.previousAltitude >= self.altitude):
				self.i_sum += 1
			else:
				self.i_sum = 0

			if self.i_sum >= self.i_stop:
				self.i_sum = self.i_stop
			elif self.i_sum <= -self.i_stop:
				self.i_sum = -self.i_stop

			controlCommand_p = (self.setPoint_p - self.altitude) * self.p			#Wanted altitude - Current altitude
			controlCommand_d = (self.setPoint_d - self.linear_acceleration_z) * self.d	#Error: Wanted acceleration - Current Acceleration
			controlCommand_i = self.i_sum * self.i

			un_cliped = self.controlCommand + controlCommand_p + controlCommand_i + controlCommand_d
			self.controlCommand = np.clip(un_cliped, 1000, 2000)

			print "Throttle: ", self.controlCommand, "Altitude: ", self.altitude, "Previous Alt: ", self.previousAltitude,  "Acceleration: ", self.linear_acceleration_z
			print "p: ", controlCommand_p, "d: ", controlCommand_d, "i: ", controlCommand_i

			self.previousAltitude = self.altitude
			self.r.sleep()

#if altitude > setPoint_p launch AltitudeHoldNode()


if __name__ == '__main__':
	try:
		controller_node = TakeOffNode()
		controller_node.run()
	except rospy.ROSInterruptException:
		pass
