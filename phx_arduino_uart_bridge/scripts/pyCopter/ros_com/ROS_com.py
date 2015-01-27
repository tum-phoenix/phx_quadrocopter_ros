__author__ = 'manuelviermetz'

import time
import numpy as np
try:
    import rospy
    import tf
    from copter_status import copter
    from sensor_msgs.msg import Imu, Quaternion
    from sensor_msgs.msg import NavSatFix
    from sensor_msgs.msg import Joy
    from sensor_msgs.msg import FluidPressure #Barometer
    from sensor_msgs.msg import Temperature #For compensation gyrodrift
    from sensor_msgs.msg import Range #Distance to ground
    from geometry_msgs.msg import Twist
    from phx_arduino_uart_bridge.msg import Motor
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue #For Battery status
except:
    print 'import rospy failed, have fun...'


class ros_communication():
    def __init__(self, copter=None):
        """
            this publishes:
            imu
            gps
            magnetometer
            rc

            this receives:
            motor messages

        """
        try:
            rospy.init_node('MultiWiiBridge')
            self.ros_publish_imu = rospy.Publisher('/phoenix/stat_imu', Imu, queue_size=10)
            self.imu_msg = Imu()
            self.ros_publish_motor = rospy.Publisher('/phoenix/stat_motor', Motor, queue_size=10)
            self.motor_msg = Motor()
            self.ros_publish_gps = rospy.Publisher('/phoenix/stat_gps', NavSatFix, queue_size=10)
            self.NavSatFix_msg = NavSatFix()
            self.ros_publish_battery = rospy.Publisher('/phoenix/stat_battery', DiagnosticArray, queue_size=10)
            self.diagnostic_msg = DiagnosticArray()
            self.ros_publish_rc0 = rospy.Publisher('/phoenix/rc_0', Joy, queue_size=10)
            self.Joy_0_msg = Joy()
            self.ros_publish_rc1 = rospy.Publisher('/phoenix/rc_1', Joy, queue_size=10)
            self.Joy_1_msg = Joy()
            self.ros_publish_rc2 = rospy.Publisher('/phoenix/rc_2', Joy, queue_size=10)
            self.Joy_2_msg = Joy()
            self.freq = 50     # Hz
            self.rate = rospy.Rate(self.freq)
        except:
            print ' >>> error in ros __init__'
        if not copter:
            print 'you have to define an copter to subscribe a ros topic'
        else:
            self.copter = copter
            # subscribe to the different topics of interest: simple_directions, commands
            self.ros_subscribe_cmd_vel = rospy.Subscriber('/phoenix/cmd_vel', Twist, self.callback_cmd_vel)
            self.ros_subscribe_cmd_motor = rospy.Subscriber('/phoenix/cmd_motor', Motor, self.callback_cmd_motor)

    def listen(self):
        self.rate.sleep()

    def callback_cmd_motor(self, motor_msg):
        print ' >>> ROS_callback: received cmd_motor', motor_msg
        self.copter

    def callback_cmd_vel(self, cmd_vel_msg):
        print ' >>> ROS_callback: received cmd_vel', cmd_vel_msg

    def pub_imu(self, acc=(1, 2, 3), gyr=(4, 5, 6), mag=(7, 8, 9), attitude=(10, 11, 12, 13), debug=False):
        """
         imu = [ acc=(accX, accY, accZ), gyr=(gyrX, gyrY, gyrZ), mag=(magX, magY. magZ), attitude=(pitch, roll, heading, altitude)
        """
        try:
            if debug: print 'trying to send imu'
            self.imu_msg.angular_velocity = gyr
            if debug: print 'imu did angular_velocity'
            self.imu_msg.linear_acceleration = acc
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
            self.NavSatFix_msg.latitude = gps_lat
            self.NavSatFix_msg.longitude = gps_lon
            self.NavSatFix_msg.altitude = gps_alt
            self.ros_publish_gps.publish(self.NavSatFix_msg)
            if debug: print ' >>> sent pub_gps'
        except:
            print '>>> error in ros pub_gps!'

    def pub_rc0(self, rc0=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
         rc0 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        try:
            if debug: print 'in pub_rc0:', rc0
            self.Joy_0_msg.axes = rc0[:4]
            if debug: print 'added first 4 axes'
            self.Joy_0_msg.buttons = rc0[4:]
            if debug: print 'added 4 buttons'
            self.ros_publish_rc0.publish(self.Joy_0_msg)
            if debug: print ' >>> sent rc0'
        except:
            print '>>> error in ros pub_rc0!'

    def pub_rc1(self, rc1=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
         rc1 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        try:
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
            self.Joy_2_msg.axes = rc2[:4]
            self.Joy_2_msg.buttons = rc2[4:]
            self.ros_publish_rc2.publish(self.Joy_2_msg)
            if debug: print ' >>> sent rc2'
        except:
            print '>>> error in ros pub_rc2!'

    def pub_battery(self, battery=(1, 2, 3, 0), debug=False):
        """
         battery = [ cell1, cell2, cell3, cell4 ]
        """
        self.diagnostic_msg.status = DiagnosticStatus(name="Battery",level=DiagnosticStatus.OK,message="OK")
        self.diagnostic_msg.status.values = [ KeyValue("Cell 1",str(battery[0])),
                                              KeyValue("Cell 2",str(battery[1])),
                                              KeyValue("Cell 3",str(battery[2])),
                                              KeyValue("Cell 4",str(battery[3]))]
        self.ros_publish_battery.publish(self.diagnostic_msg)


if __name__ == '__main__':
    try:
        ros_network = ros_communication()

        while True:
            # do some stuff like updating data via serial connection
            time.sleep(0.5)

            ros_network.pub_motors([2, 4, 6, 8])

    except rospy.ROSInterruptException:
        pass
