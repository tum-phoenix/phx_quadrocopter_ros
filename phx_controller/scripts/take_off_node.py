'''
#!usr/bin/env python
import numpy as np
import rospy
import time
from altitude_hold_node import AltitudeHoldNode
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import RemoteControl
from sensor_msgs.msg import Joy

class TakeOffNode():
	def __init__(self):
		rospy.init_node('take_off_controller')
		self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
		self.altitude_pub = rospy.Publisher('/phx/fc/altitude_hold', RemoteControl, queue_size=1)
		self.sub = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, self.altitudeCallback)

		self.throttle = 1000
		self.finish_take_off = 1
		self.flag = 0

		self.freq = 100
		self.r = rospy.Rate(self.freq)

	def run(self):
		while not rospy.is_shutdown():
			self.r.sleep()

	def altitudeCallback(self, altitude_msg):
        	print("Altitude Callback")


		joy_msg = RemoteControl()

		joy_msg.pitch = self.input_rc[0]
                joy_msg.roll = self.input_rc[1]
                joy_msg.yaw = self.input_rc[2]
		joy_msg.throttle = self.throttle
                joy_msg.aux1 = self.input_rc[4]
                joy_msg.aux2 = self.input_rc[5]
                joy_msg.aux3 = self.input_rc[6]
                joy_msg.aux4 = self.input_rc[7]

		self.altitude_pub.publish(joy_msg)

		if altitude_msg.estimated_altitude > self.finish_take_off:
			print "Take-off finished"

			
		else:
			print "Take-off in progress"
			if self.throttle < 2000 and self.flag == 0:
				if altitude_msg.estimated_altitude < 0.2:
					self.throttle = self.throttle + 10
					time.sleep(0.1)


			print self.throttle

		print 'Throttle:', joy_msg.throttle, '\t Altitude:', altitude_msg.estimated_altitude

if __name__ == '__main__':
	try:
		controller_node = TakeOffNode()
		controller_node.run()
	except rospy.ROSInterruptException:
		pass

'''

#!usr/bin/env python
import numpy as np
import rospy
import time
from altitude_hold_node import AltitudeHoldNode
from phx_uart_msp_bridge.msg import Altitude
from phx_uart_msp_bridge.msg import RemoteControl
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu

#it's possible that I Control is not necessary for the program to fulfill its task

class TakeOffNode():
	def __init__(self):
		rospy.init_node('take_off_controller')

		self.sub_alt = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, self.altCallback)
	        self.sub_imu = rospy.Subscriber('/phx/imu', Imu, self.imuCallback)

		self.p = 10	#PID controller
		self.d = 5
		self.i = 1
		self.i_sum = 0
		self.i_stop = 1

		self.setPoint_p = 1	#wanted altitude
		self.setPoint_d = 10	#wanted acceleration

		self.altitude = 0
		self.linear_acceleration_z = 0

		self.controlCommand = 1500	#throttle
		self.previousAltitude = 0

		self.freq = 100
		self.r = rospy.Rate(self.freq)

#Following two functions read altitude and acceleration from the sensors

	def altCallback(self, alt_msg):
		self.altitude = alt_msg.estimated_altitude
		#self.altitude = 1

	def imuCallback(self, imu_msg):
		self.linear_acceleration_z = imu_msg.linear_acceleration.z
		#self.linear_acceleration_z = 5

	def run(self):
		while not rospy.is_shutdown():

			self.i_sum += self.previousAltitude - self.altitude	#Maybe not necessary
			self.previousAltitude = self.altitude

			if self.i_sum >= self.i_stop:
				self.i_sum = self.i_stop
			elif self.i_sum <= -self.i_stop:
				self.i_sum = -self.i_stop

			controlCommand_p = (self.setPoint_p - self.altitude) * self.p			#Wanted altitude - Current altitude
			controlCommand_d = (self.setPoint_d - self.linear_acceleration_z) * self.d	#Error: Wanted acceleration - Current Acceleration
			controlCommand_i = self.i_sum * self.i

			un_cliped = self.controlCommand + controlCommand_p + controlCommand_i + controlCommand_d
			self.controlCommand = np.clip(un_cliped, 1000, 2000)

			print(self.controlCommand, self.altitude, self.linear_acceleration_z)
			self.r.sleep()

#if altitude > setPoint_p launch AltitudeHoldNode()


if __name__ == '__main__':
	try:
		controller_node = TakeOffNode()
		controller_node.run()
	except rospy.ROSInterruptException:
		pass
