#!usr/bin/env python
import numpy as np
import rospy
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

		self.throttle = 2000
		self.finish_take_off = 0.1

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
			rospy.signal_shutdown("Done.")
			#After finishing the process, altitude controller should be launched.
		else:
			print "Take-off in progress"

		print 'Throttle:', joy_msg.throttle, '\t Altitude:', altitude_msg.estimated_altitude


if __name__ == '__main__':
	try:
		controller_node = TakeOffNode()
		controller_node.run()
	except rospy.ROSInterruptException:
		pass
