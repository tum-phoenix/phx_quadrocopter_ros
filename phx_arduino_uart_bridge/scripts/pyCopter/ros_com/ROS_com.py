__author__ = 'manuelviermetz'
# based on https://github.com/tum-phoenix/phx_controller/blob/master/src/controller_node.py

import time
import numpy as np

import rospy
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from sensor_msgs.msg import FluidPressure #Barometer
from sensor_msgs.msg import Temperature #For compensation gyrodrift
from sensor_msgs.msg import Range #Distance to ground
from geometry_msgs.msg import Twist, Quaternion
from phx_arduino_uart_bridge.msg import Motor
from phx_arduino_uart_bridge.msg import Battery
from phx_arduino_uart_bridge.msg import Cycletime
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue #For Battery status


class ros_communication():
    def __init__(self, copter=None, osc=None):
        """
            If copter is defined this is either a node which is connected to multiwii
                publish stat_imu
                publish stat_motor
                publish stat_gps
                publish stat_rc2        (multiwii rc for feedback)
                subscribe com_motor     (not fully implemented jet)
            or connected to the marvic
                publish stat_battery
                publish stat_rc0        (sudo rc)
                publish stat_rc1        (computer rc for feedback)
                subscribe com_rc1       (computer rc)
                subscribe com_vel
            In case osc is defined the node will
                publish com_vel         (sent by the base station)
                publish com_rc1         (sent by the base station)
                subscribe stat_imu
                subscribe stat_motor
                subscribe stat_gps
                subscribe stat_battery
                subscribe stat_rc0
                subscribe stat_rc1
                subscribe stat_rc2
        """
        try:
            if copter:
                if copter.serial_multiwii and not copter.serial_intermediate:
                    rospy.init_node('MultiWii_Bridge')
                    self.ros_publish_imu = rospy.Publisher('/phoenix/stat_imu', Imu, queue_size=10)
                    self.imu_msg = Imu()
                    self.ros_publish_motor = rospy.Publisher('/phoenix/stat_motor', Motor, queue_size=10)
                    self.motor_msg = Motor()
                    self.ros_publish_gps = rospy.Publisher('/phoenix/stat_gps', NavSatFix, queue_size=10)
                    self.NavSatFix_msg = NavSatFix()
                    self.ros_publish_rc2 = rospy.Publisher('/phoenix/stat_rc2', Joy, queue_size=10)
                    self.Joy_2_msg = Joy()
                    self.ros_publish_cycletime0 = rospy.Publisher('/phoenix/cycletime0', Cycletime, queue_size=10)
                    self.cycletime_0_msg = Cycletime()
                elif copter.serial_intermediate and not copter.serial_multiwii:
                    rospy.init_node('MARVIC_Bridge')
                    self.ros_publish_battery = rospy.Publisher('/phoenix/stat_battery', Battery, queue_size=10)
                    self.Battery = Battery()
                    self.ros_publish_rc0 = rospy.Publisher('/phoenix/stat_rc0', Joy, queue_size=10)
                    self.Joy_0_msg = Joy()
                    self.ros_publish_rc1 = rospy.Publisher('/phoenix/stat_rc1', Joy, queue_size=10)
                    self.Joy_1_msg = Joy()
                    self.ros_publish_cycletime1 = rospy.Publisher('/phoenix/cycletime1', Cycletime, queue_size=10)
                    self.cycletime_1_msg = Cycletime()
                elif copter.serial_intermediate and copter.serial_multiwii:
                    rospy.init_node('MultiWii_MARVIC_Bridge')
                    self.ros_publish_imu = rospy.Publisher('/phoenix/stat_imu', Imu, queue_size=10)
                    self.imu_msg = Imu()
                    self.ros_publish_motor = rospy.Publisher('/phoenix/stat_motor', Motor, queue_size=10)
                    self.motor_msg = Motor()
                    self.ros_publish_gps = rospy.Publisher('/phoenix/stat_gps', NavSatFix, queue_size=10)
                    self.NavSatFix_msg = NavSatFix()
                    self.ros_publish_battery = rospy.Publisher('/phoenix/stat_battery', Battery, queue_size=10)
                    self.Battery = Battery()
                    self.ros_publish_rc0 = rospy.Publisher('/phoenix/stat_rc0', Joy, queue_size=10)
                    self.Joy_0_msg = Joy()
                    self.ros_publish_rc1 = rospy.Publisher('/phoenix/stat_rc1', Joy, queue_size=10)
                    self.Joy_1_msg = Joy()
                    self.ros_publish_rc2 = rospy.Publisher('/phoenix/stat_rc2', Joy, queue_size=10)
                    self.Joy_2_msg = Joy()
                    self.ros_publish_cycletime0 = rospy.Publisher('/phoenix/cycletime0', Cycletime, queue_size=10)
                    self.cycletime_0_msg = Cycletime()
                    self.ros_publish_cycletime1 = rospy.Publisher('/phoenix/cycletime1', Cycletime, queue_size=10)
                    self.cycletime_1_msg = Cycletime()
                self.copter = copter
                self.osc_transmitter = None
                # subscribe to the different topics of interest: simple_directions, commands
                if copter.serial_multiwii:
                    self.ros_subscribe_cmd_motor = rospy.Subscriber('/phoenix/cmd_motor', Motor, self.callback_cmd_motor)
                if copter.serial_intermediate:
                    self.ros_subscribe_cmd_vel = rospy.Subscriber('/phoenix/cmd_vel', Twist, self.callback_cmd_vel)
                    self.ros_subscribe_cmd_rc_1 = rospy.Subscriber('/phoenix/cmd_rc1', Joy, self.callback_cmd_rc_1)
            elif osc:
                rospy.init_node('OSC_Bridge')
                self.ros_publish_cmd_rc1 = rospy.Publisher('/phoenix/cmd_rc1', Joy, queue_size=10)
                self.Joy_1_cmd_msg = Joy()
                self.ros_publish_cmd_vel = rospy.Publisher('/phoenix/cmd_vel', Joy, queue_size=10)
                self.cmd_vel_msg = Twist()
                self.ros_subscribe_stat_imu = rospy.Subscriber('/phoenix/stat_imu', Imu, self.callback_stat_imu)
                self.ros_subscribe_stat_motor = rospy.Subscriber('/phoenix/stat_motor', Motor, self.callback_stat_motor)
                self.ros_subscribe_stat_gps = rospy.Subscriber('/phoenix/stat_gps', NavSatFix, self.callback_stat_gps)
                self.ros_subscribe_stat_battery = rospy.Subscriber('/phoenix/stat_battery', Battery, self.callback_stat_battery)
                self.ros_subscribe_stat_rc0 = rospy.Subscriber('/phoenix/stat_rc0', Joy, self.callback_stat_rc0)
                self.ros_subscribe_stat_rc1 = rospy.Subscriber('/phoenix/stat_rc1', Joy, self.callback_stat_rc1)
                self.ros_subscribe_stat_rc2 = rospy.Subscriber('/phoenix/stat_rc2', Joy, self.callback_stat_rc2)
                self.ros_subscribe_stat_cycletime0 = rospy.Subscriber('/phoenix/cycletime0', Int16, self.callback_stat_cycletime0)
                self.ros_subscribe_stat_cycletime1 = rospy.Subscriber('/phoenix/cycletime1', Int16, self.callback_stat_cycletime1)
                self.copter = None
                self.debug_osc = True
                self.osc_transmitter = osc
            self.freq = 50     # Hz
            self.rate = rospy.Rate(self.freq)
        except:
            print ' >>> error in ros __init__'
        """
            raw values:
            throttle           0  -   100
            pitch roll yaw  (-50) -   +50
            aux1 to aux4       0  -   100
        """
        self.throttle = 20
        self.pitch = 20
        self.roll = 30
        self.yaw = 30
        self.aux1 = 40
        self.aux2 = 50
        self.aux3 = 60
        self.aux4 = 70
        """
            pwm values:
            pwm_* is from 1000 to 2000
        """
        self.pwm_throttle = 0
        self.pwm_pitch = 0
        self.pwm_roll = 0
        self.pwm_yaw = 0
        self.pwm_aux1 = 0
        self.pwm_aux2 = 0
        self.pwm_aux3 = 0
        self.pwm_aux4 = 0

        # pwm midpoints:
        self.pwm_midpoint_throttle = 1000
        self.pwm_midpoint_pitch = 1500
        self.pwm_midpoint_roll = 1500
        self.pwm_midpoint_yaw = 1500
        self.pwm_midpoint_aux1 = 1000
        self.pwm_midpoint_aux2 = 1000
        self.pwm_midpoint_aux3 = 1000
        self.pwm_midpoint_aux4 = 1000

        self.simple_directions = [0, 0, 0, 0]   # backward-forward, left-right, up-down, left-right-turn
        self.simple_directions_linear = [10, 10, 10, 10]

    def listen(self):
        """
            this makes rospy look for incoming messages which will be received by their callback functions.
        """
        self.rate.sleep()

    # Callbacks:
    def callback_stat_imu(self, stuff):
        if self.osc_transmitter:
            # TODO: organize the imu stuff and send it
            # imu = [ accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY. magZ, pitch, roll, heading, altitude ]
            # self.osc_transmitter.send_imu(imu=(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13), debug=False):
            attitude = tf.transformations.euler_from_quaternion([stuff.orientation.x, stuff.orientation.y, stuff.orientation.z, stuff.orientation.w])
            imu = [stuff.linear_acceleration.x, stuff.linear_acceleration.y, stuff.linear_acceleration.z,
                   stuff.angular_velocity.x, stuff.angular_velocity.y, stuff.angular_velocity.z,
                   0, 0, 0,
                   attitude[0], attitude[1], attitude[2], 0, 0]
            print ' > imu:', imu
#        print ' > not implemented jet, imu', stuff

    def callback_stat_motor(self, stuff):
        if self.osc_transmitter:
            # TODO: organize the motor stuff and send it
            # motors = [ motor0, motor1, motor2, motor3 ]
            # self.osc_transmitter.send_motors(motors=(1, 2, 3, 4), debug=False):
            motors = [stuff.motor0, stuff.motor1, stuff.motor2, stuff.motor3]
            print ' > motors:', motors
#        print ' > not implemented jet, motor', stuff

    def callback_stat_gps(self, stuff):
        if self.osc_transmitter:
            # TODO: organize the gps stuff and send it
            # This has to be implemented in osc_com first.
            latitude = stuff.latitude
            longitude = stuff.longitude
            altitude = stuff.altitude
#        print ' > not implemented jet, gps', stuff

    def callback_stat_battery(self, stuff):
        if self.osc_transmitter:
            # TODO: organize the battery stuff and send it
            # battery = [ cell1, cell2, cell3, cell4 ]
            battery = [stuff.cell1, stuff.cell2, stuff.cell3, stuff.cell4]
            print ' > battery:', battery
            # self.osc_transmitter.send_battery(battery=(1, 2, 3, 0), debug=False):
#        print ' > not implemented jet, battery', stuff

    def callback_stat_rc0(self, stuff):
        if self.osc_transmitter:
            # TODO: organize the rc0 stuff and send it
            # rc0 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
            rc0 = [stuff.axes[0], stuff.axes[1], stuff.axes[2], stuff.axes[3], stuff.buttons[0], stuff.buttons[1], stuff.buttons[2], stuff.buttons[3]]
            print ' > rc0:', rc0
            self.osc_transmitter.send_rc0(rc0, debug=self.debug_osc)
            print ' >> rc0 sent'
            # self.osc_transmitter.send_rc0(rc0=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
#        print ' > not implemented jet, rc0', stuff.axes

    def callback_stat_rc1(self, stuff):
        if self.osc_transmitter:
            # TODO: organize the rc1 stuff and send it
            # rc1 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
            # self.osc_transmitter.send_rc1(rc1=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
            rc1 = [stuff.axes[0], stuff.axes[1], stuff.axes[2], stuff.axes[3], stuff.buttons[0], stuff.buttons[1], stuff.buttons[2], stuff.buttons[3]]
            print ' > rc1:', rc1
            self.osc_transmitter.send_rc1(rc1, debug=self.debug_osc)
            print ' >> rc1 sent'
#        print ' > not implemented jet, rc1', stuff

    def callback_stat_rc2(self, stuff):
        if self.osc_transmitter:
            # TODO: organize the rc2 stuff and send it
            # rc2 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
            # self.osc_transmitter.send_rc2(rc2=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
            rc2 = [stuff.axes[0], stuff.axes[1], stuff.axes[2], stuff.axes[3], stuff.buttons[0], stuff.buttons[1], stuff.buttons[2], stuff.buttons[3]]
            print ' > rc2:', rc2
            self.osc_transmitter.send_rc2(rc2, debug=self.debug_osc)
            print ' >> rc2 sent'
#        print ' > not implemented jet, rc2', stuff

    def callback_stat_cycletime0(self, stuff):
        if self.osc_transmitter:
            # TODO: organize the cycletime0 stuff and send it
            # self.osc_transmitter.send_cycletime1(cycletime_1, debug=False):
            pass
        print ' > not implemented jet, cycletime0', stuff

    def callback_stat_cycletime1(self, stuff):
        if self.osc_transmitter:
            # TODO: organize the cycletime1 stuff and send it
            # self.osc_transmitter.send_cycletime1(cycletime_1, debug=False)
            pass
        print ' > not implemented jet, cycletime1', stuff

    def callback_cmd_motor(self, stuff):
        """
            This callback is used for incoming motor commands and it will directly send them to the copter!
        """
        print ' >>> ROS_callback: received cmd_motor', stuff
        # TODO: link this correctly
        motors = [stuff.motor0, stuff.motor1, stuff.motor2, stuff.motor3]
        print motors
        # probably like this
        if self.copter:
            self.copter.send_serial_motor(motor_values=motors)
        else:
            print 'no copter connected -> no motor commands sent'

    def callback_cmd_vel(self, stuff):
        """
            This callback is used for incoming com_vel commands and updates the ros RC to the new state.
        """
        print ' >>> ROS_callback: received cmd_vel', stuff
        # TODO: link this correctly
        self.simple_directions = stuff
        self.calc_rc_from_simple_directions()
        print '     >>> not implemented jet'

    def callback_cmd_rc_1(self, stuff):
        """
            in case stuff = [ self.throttle, self.pitch, self.roll, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4 ]
            This callback is used for incoming rc_1 commands and updates the ros RC to the new state.
        """
        try:
            print stuff.axes
            print stuff.buttons
            self.set_sticks(sticks=stuff)
        except:
            print ' >>> ROS_callback: receive callback_cmd_rc_1 failed', stuff

    # Publishers:
    def pub_imu(self, acc=(1, 2, 3), gyr=(4, 5, 6), mag=(7, 8, 9), attitude=(10, 11, 12, 13), debug=False):
        """
            acc=(accX, accY, accZ), gyr=(gyrX, gyrY, gyrZ), mag=(magX, magY. magZ), attitude=(pitch, roll, heading, altitude)
        """
        try:
            if debug: print 'trying to send imu'
            
            now = rospy.get_rostime()
            self.imu_msg.header.stamp.secs = now.secs
            self.imu_msg.header.stamp.nsecs = now.nsecs
            self.imu_msg.angular_velocity.x = gyr[0]
            self.imu_msg.angular_velocity.y = gyr[1]
            self.imu_msg.angular_velocity.z = gyr[2]
            if debug: print 'imu did angular_velocity'
            self.imu_msg.linear_acceleration.x = acc[0]
            self.imu_msg.linear_acceleration.y = acc[1]
            self.imu_msg.linear_acceleration.z = acc[2]
            if debug: print 'imu did linear_acceleration'
            q = tf.transformations.quaternion_from_euler(attitude[0], attitude[1], attitude[2])
            self.imu_msg.orientation = Quaternion(*q)
            if debug: print 'imu did orientation'
            self.ros_publish_imu.publish(self.imu_msg)
            if debug: print ' >>> sent imu'
        except:
            print '>>> error in ros pub_imu!'

    def pub_motors(self, motors=(1, 2, 3, 4), debug=False):
        """
            motors = [ motor0, motor1, motor2, motor3 ]
        """
        try:
            now = rospy.get_rostime()
            self.motor_msg.header.stamp.secs = now.secs
            self.motor_msg.header.stamp.nsecs = now.nsecs
            self.motor_msg.motor0 = motors[0]
            self.motor_msg.motor1 = motors[1]
            self.motor_msg.motor2 = motors[2]
            self.motor_msg.motor3 = motors[3]
            self.ros_publish_motor.publish(self.motor_msg)
            if debug: print ' >>> sent rc2'
        except:
            print '>>> error in ros pub_motor!'

    def pub_gps(self, gps_lat, gps_lon, gps_alt, debug=False):
        try:
            now = rospy.get_rostime()
            self.NavSatFix_msg.header.stamp.secs = now.secs
            self.NavSatFix_msg.header.stamp.nsecs = now.nsecs
            self.NavSatFix_msg.latitude = gps_lat
            self.NavSatFix_msg.longitude = gps_lon
            self.NavSatFix_msg.altitude = gps_alt
            self.ros_publish_gps.publish(self.NavSatFix_msg)
            if debug: print ' >>> sent pub_gps'
        except:
            print '>>> error in ros pub_gps!'

    def pub_battery(self, battery=(1, 2, 3, 0), debug=False):
        """
            battery = [ cell1, cell2, cell3, cell4 ]
        """
        try:
            print 'battery:', battery
            now = rospy.get_rostime()
            self.Battery.header.stamp.secs = now.secs
            self.Battery.header.stamp.nsecs = now.nsecs
            self.Battery.cell1 = battery[0]
            self.Battery.cell2 = battery[1]
            self.Battery.cell3 = battery[2]
            self.Battery.cell4 = battery[3]
            self.ros_publish_battery.publish(self.Battery)
            if debug: print ' >>> sent pub_battery'
        except:
            print '>>> error in ros pub_battery!'
    
    def pub_cycletime0(self, cycletime0, debug=False):
        try:
            now = rospy.get_rostime()
            self.cycletime_0_msg.header.stamp.secs = now.secs
            self.cycletime_0_msg.header.stamp.nsecs = now.nsecs
            self.cycletime_0_msg.cycletime = cycletime0
            self.ros_publish_cycletime0.publish(self.cycletime_0_msg)
            if debug: print ' >>> sent pub_cycletime_0'
        except:
            print '>>> error in ros pub_cycletime_0!', cycletime0
    
    def pub_cycletime1(self, cycletime1, debug=False):
        try:
            now = rospy.get_rostime()
            self.cycletime_1_msg.header.stamp.secs = now.secs
            self.cycletime_1_msg.header.stamp.nsecs = now.nsecs
            self.cycletime_1_msg.cycletime = cycletime1
            self.ros_publish_cycletime1.publish(self.cycletime_1_msg)
            if debug: print ' >>> sent pub_cycletime_1'
        except:
            print '>>> error in ros pub_cycletime_1!', cycletime1

    def pub_rc0(self, rc0=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
            rc0 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        try:
            now = rospy.get_rostime()
            self.Joy_0_msg.header.stamp.secs = now.secs
            self.Joy_0_msg.header.stamp.nsecs = now.nsecs
            self.Joy_0_msg.axes = rc0[:4]
            self.Joy_0_msg.buttons = rc0[4:]
            self.ros_publish_rc0.publish(self.Joy_0_msg)
            if debug: print ' >>> sent rc0'
        except:
            print '>>> error in ros pub_rc0!'

    def pub_rc1(self, rc1=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
            rc1 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        try:
            now = rospy.get_rostime()
            self.Joy_1_msg.header.stamp.secs = now.secs
            self.Joy_1_msg.header.stamp.nsecs = now.nsecs
            self.Joy_1_msg.axes = rc1[:4]
            self.Joy_1_msg.buttons = rc1[4:]
            self.ros_publish_rc1.publish(self.Joy_1_msg)
            if debug: print ' >>> sent rc1'
        except:
            print '>>> error in ros pub_rc1!'

    def pub_rc2(self, rc2=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
         rc2 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        try:
            now = rospy.get_rostime()
            self.Joy_2_msg.header.stamp.secs = now.secs
            self.Joy_2_msg.header.stamp.nsecs = now.nsecs
            self.Joy_2_msg.axes = rc2[:4]
            self.Joy_2_msg.buttons = rc2[4:]
            self.ros_publish_rc2.publish(self.Joy_2_msg)
            if debug: print ' >>> sent rc2'
        except:
            print '>>> error in ros pub_rc2!'

    def pub_cmd_motor(self, motors=(0, 0, 0, 0), debug=False):
        # TODO: write this publisher! very important for testing!
        print 'not implemented jet, pub_cmd_motor', motors

    def pub_cmd_vel(self, simple_directions=(0, 0, 0, 0), debug=False):
        # TODO: write this publisher! very important for testing!
        print 'not implemented jet, pub_cmd_vel', simple_directions

    def pub_cmd_rc1(self, rc1=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
            rc1 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        try:
            self.Joy_1_cmd_msg.axes = rc1[:4]
            self.Joy_1_cmd_msg.buttons = rc1[4:]
            self.ros_publish_cmd_rc1.publish(self.Joy_1_cmd_msg)
            if debug: print ' >>> sent rc1'
        except:
            print '>>> error in ros pub_rc1!'

    # RC stuff:
    def update_rc(self, debug=False):
        if self.copter:
            self.copter.send_serial_rc(remote_control=self, debug=debug)

    def stick_curve(self, raw_val, mid_point=1500, mode=0):
        if mode == 0:
            return mid_point + 10. * raw_val
        else:
            return mid_point + 10. * raw_val

    def validate(self):
        lower_end = 0
        upper_end = 100
        if self.throttle < lower_end:   self.throttle = lower_end
        if self.aux1 < lower_end:       self.aux1 = lower_end
        if self.aux2 < lower_end:       self.aux2 = lower_end
        if self.aux3 < lower_end:       self.aux3 = lower_end
        if self.aux4 < lower_end:       self.aux4 = lower_end
        if self.throttle > upper_end:   self.throttle = upper_end
        if self.aux1 > upper_end:       self.aux1 = upper_end
        if self.aux2 > upper_end:       self.aux2 = upper_end
        if self.aux3 > upper_end:       self.aux3 = upper_end
        if self.aux4 > upper_end:       self.aux4 = upper_end

        lower_end = -50
        upper_end = 50
        if self.pitch < lower_end:      self.pitch = lower_end
        if self.roll < lower_end:       self.pitch = lower_end
        if self.yaw < lower_end:        self.pitch = lower_end
        if self.pitch > upper_end:      self.pitch = upper_end
        if self.roll > upper_end:       self.pitch = upper_end
        if self.yaw > upper_end:        self.pitch = upper_end

    def validate_pwm(self):
        lower_end = 1000
        upper_end = 2000
        if self.pwm_throttle < lower_end:   self.pwm_throttle = lower_end
        if self.pwm_pitch < lower_end:      self.pwm_pitch = lower_end
        if self.pwm_roll < lower_end:       self.pwm_pitch = lower_end
        if self.pwm_yaw < lower_end:        self.pwm_pitch = lower_end
        if self.pwm_aux1 < lower_end:       self.pwm_aux1 = lower_end
        if self.pwm_aux2 < lower_end:       self.pwm_aux2 = lower_end
        if self.pwm_aux3 < lower_end:       self.pwm_aux3 = lower_end
        if self.pwm_aux4 < lower_end:       self.pwm_aux4 = lower_end
        if self.pwm_throttle > upper_end:   self.pwm_throttle = upper_end
        if self.pwm_pitch > upper_end:      self.pwm_pitch = upper_end
        if self.pwm_roll > upper_end:       self.pwm_pitch = upper_end
        if self.pwm_yaw > upper_end:        self.pwm_pitch = upper_end
        if self.pwm_aux1 > upper_end:       self.pwm_aux1 = upper_end
        if self.pwm_aux2 > upper_end:       self.pwm_aux2 = upper_end
        if self.pwm_aux3 > upper_end:       self.pwm_aux3 = upper_end
        if self.pwm_aux4 > upper_end:       self.pwm_aux4 = upper_end

    def calc_rc_from_simple_directions(self, throttle=None, debug=False):
        """
            setting the raw values based on self.simple_directions
            simple_directions = [0, 0, 0, 0]   # backward-forward, left-right, up-down, left-right-turn
        """
        if not throttle:
            self.throttle = self.throttle
        else:
            self.throttle = throttle

        self.pitch = self.simple_directions[0] * self.simple_directions_linear[0]
        if abs(self.pitch) > 25:
            self.pitch = 25.0 * (self.pitch/abs(self.pitch))

        self.roll = self.simple_directions[1] * self.simple_directions_linear[1]
        if abs(self.roll) > 25:
            self.roll = 25.0 * (self.roll/abs(self.roll))

        self.throttle += self.simple_directions[3] * self.simple_directions_linear[3]

        self.yaw = self.simple_directions[3] * self.simple_directions_linear[3]
        if abs(self.yaw) < 10:
            self.yaw = 0
        elif abs(self.yaw) > 35:
            self.yaw = 30

    def calc_pwm_from_raw(self, debug=False):
        if debug: print ' >>> calc_pwm_from_raw()'
        if debug: print 'before: raw ', self.throttle, self.pitch, self.roll, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4
        if debug: print 'before: pwm ', self.pwm_throttle, self.pwm_pitch, self.pwm_roll, self.pwm_yaw, self.pwm_aux1, self.pwm_aux2, self.pwm_aux3, self.pwm_aux4
        self.validate()
        self.pwm_throttle = self.stick_curve(self.throttle, mid_point=self.pwm_midpoint_throttle, mode=0)
        self.pwm_pitch = self.stick_curve(self.pitch, mid_point=self.pwm_midpoint_pitch, mode=0)
        self.pwm_roll = self.stick_curve(self.roll, mid_point=self.pwm_midpoint_roll, mode=0)
        self.pwm_yaw = self.stick_curve(self.yaw, mid_point=self.pwm_midpoint_yaw, mode=0)
        self.pwm_aux1 = self.stick_curve(self.aux1, mid_point=self.pwm_midpoint_aux1, mode=0)
        self.pwm_aux2 = self.stick_curve(self.aux2, mid_point=self.pwm_midpoint_aux2, mode=0)
        self.pwm_aux3 = self.stick_curve(self.aux3, mid_point=self.pwm_midpoint_aux3, mode=0)
        self.pwm_aux4 = self.stick_curve(self.aux4, mid_point=self.pwm_midpoint_aux4, mode=0)
        self.validate_pwm()
        if debug: print 'after: raw ', self.throttle, self.pitch, self.roll, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4
        if debug: print 'after: pwm ', self.pwm_throttle, self.pwm_pitch, self.pwm_roll, self.pwm_yaw, self.pwm_aux1, self.pwm_aux2, self.pwm_aux3, self.pwm_aux4

    def get_pwm_sticks(self, update_from_raw_sticks=True):
        """
            returns one pwm stick position on a scale between 1000 and 1500
        """
        if update_from_raw_sticks:
            self.calc_pwm_from_raw()
        return [self.pwm_throttle, self.pwm_pitch, self.pwm_roll, self.pwm_yaw, self.pwm_aux1, self.pwm_aux2, self.pwm_aux3, self.pwm_aux4]

    def get_pwm_stick(self, name, update_from_raw_sticks=True):
        """
            returns one pwm stick position on a scale between 1000 and 2000
        """
        if update_from_raw_sticks:
            self.calc_pwm_from_raw()
        if name is 'throttle':
            return self.pwm_throttle
        elif name is 'pitch':
            return self.pwm_pitch
        elif name is 'roll':
            return self.pwm_roll
        elif name is 'yaw':
            return self.pwm_yaw
        elif name is 'aux1':
            return self.pwm_aux1
        elif name is 'aux2':
            return self.pwm_aux2
        elif name is 'aux3':
            return self.pwm_aux3
        elif name is 'aux4':
            return self.pwm_aux4
        else:
            print ' >>> virtual_remote_control get_pwm_stick(', name, ') request could not be answered correctly since this stick name is not available'
            return 1500.

    def get_sticks(self):
        """
            returns the raw stick positions on a scale between 0 and 100 or -50 to 50
        """
        return [self.throttle, self.pitch, self.roll, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4]

    def get_stick(self, name):
        """
            returns one raw stick position on a scale between 0 and 100 or -50 to 50
        """
        if name is 'throttle':
            return self.throttle
        elif name is 'pitch':
            return self.pitch
        elif name is 'roll':
            return self.roll
        elif name is 'yaw':
            return self.yaw
        elif name is 'aux1':
            return self.aux1
        elif name is 'aux2':
            return self.aux2
        elif name is 'aux3':
            return self.aux3
        elif name is 'aux4':
            return self.aux4
        else:
            print ' >>> virtual_remote_control get_stick(', name, ') request could not be answered correctly since this stick name is not available'
            return 50.

    def set_sticks(self, sticks, debug=False):
        """
            sets the raw sticks on a scale between 0 and 100 or -50 to 50
        """
        if len(sticks) == 8:
            if debug: print ' >>> setting sticks to:', sticks
            self.throttle, self.pitch, self.roll, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4 = sticks
        else:
            print ' >>> set_sticks with wrong shape!', sticks

    def set_stick(self, name, value, debug=False):
        """
            sets one raw stick on a scale between 0 and 100 or -50 to 50
        """
        if name is 'throttle':
            self.throttle = value
        elif name is 'pitch':
            self.pitch = value
        elif name is 'roll':
            self.roll = value
        elif name is 'yaw':
            self.yaw = value
        elif name is 'aux1':
            self.aux1 = value
        elif name is 'aux2':
            self.aux2 = value
        elif name is 'aux3':
            self.aux3 = value
        elif name is 'aux4':
            self.aux4 = value
        else:
            print ' >>> virtual_remote_control set_stick(', name, ') could not be set since this stick name is not available'

    def handle_gyro(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/gyro fff [0.48758959770202637, 0.06476165354251862, -0.19856473803520203] ('192.168.0.33', 57527)
        self.gyro = stuff

    def handle_acc(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/accel fff [0.48758959770202637, 0.06476165354251862, -0.19856473803520203] ('192.168.0.3$
        self.acc = stuff

    def handle_gps(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/gps fff
        self.gps = stuff

    def handle_ip(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/ipport fff
        self.ip = stuff

    def handle_directions(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/ipport fff
        print 'received directions:', stuff, 'this is printed by ROS.handle_directions() needs further implementation'
        self.simple_directions = stuff
        self.pub_cmd_vel(simple_directions=stuff)

    def handle_commands(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/ipport fff
        print 'received commands:', stuff

if __name__ == '__main__':
    try:
        ros_network = ros_communication()

        while True:
            # do some stuff like updating data via serial connection
            time.sleep(0.5)

            ros_network.pub_motors([2, 4, 6, 8])

    except rospy.ROSInterruptException:
        pass
