import numpy as np
import rospy
from phx_uart_msp_bridge.msg import Attitude
from phx_uart_msp_bridge.msg import RemoteControl
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu


class AttitudeHoldNode():
    def __init__(self):
        rospy.init_node('attitude_hold_controller')
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.sub_imu = rospy.Subscriber('/phx/imu', Joy, self.imuCallback)
        self.sub_attitude = rospy.Subscriber('/phx/fc/attitude', Attitude, self.attitudeCallback)
        self.pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)

        self.imu = Imu()

        self.freq = 100  # Hz
        self.r = rospy.Rate(self.freq)

        self.currentPose = Attitude()
        self.set_pitch = 0
        self.set_roll = 0
        self.set_yaw = 0

        self.pitch_p = 1
        self.pitch_d = 4
        self.pitch_setPoint_d = 0
        self.pitch_i = 0
        self.pitch_sum_i = 0
        self.pitch_i_stop = 100

        self.roll_p = 1
        self.roll_d = 4
        self.roll_setPoint_d = 0
        self.roll_i = 0
        self.roll_sum_i = 0
        self.roll_i_stop = 100

        self.controlCommand_pitch = 1500
        self.controlCommand_roll = 1500
        self.controlCommand_yaw = 1500

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def attitudeCallback(self, attitude_msg):
        self.currentPose = attitude_msg

        self.pitch_sum_i += self.set_pitch - attitude_msg.pitch

        if self.pitch_sum_i >= self.pitch_i_stop:
            self.pitch_sum_i = self.pitch_i_stop
        elif self.pitch_sum_i <= -self.pitch_i_stop:
            self.pitch_sum_i = -self.pitch_i_stop
        pitch_controlCommand_p = (self.set_pitch - attitude_msg.pitch) * self.pitch_p
        pitch_controlCommand_d = (self.pitch_setPoint_d - self.imu.angular_velocity.y) * self.pitch_d
        pitch_controlCommand_i = self.pitch_sum_i * self.pitch_i
        un_cliped = self.controlCommand_pitch + pitch_controlCommand_p + pitch_controlCommand_d + pitch_controlCommand_i
        self.controlCommand_pitch = np.clip(un_cliped, 1000, 2000)

        self.roll_sum_i += self.set_roll - attitude_msg.roll

        if self.roll_sum_i >= self.roll_i_stop:
            self.roll_sum_i = self.roll_i_stop
        elif self.roll_sum_i <= -self.roll_i_stop:
            self.roll_sum_i = -self.roll_i_stop
        roll_controlCommand_p = (self.set_roll - attitude_msg.roll) * self.roll_p
        roll_controlCommand_d = (self.roll_setPoint_d - self.imu.angular_velocity.y) * self.roll_d
        roll_controlCommand_i = self.roll_sum_i * self.roll_i
        un_cliped = self.controlCommand_roll + roll_controlCommand_p + roll_controlCommand_d + roll_controlCommand_i
        self.controlCommand_roll = np.clip(un_cliped, 1000, 2000)


        joy_msg = RemoteControl()
        joy_msg.pitch = self.controlCommand_pitch
        joy_msg.roll = self.controlCommand_roll

        self.pub.publish(joy_msg)

        #print 'cc: ', self.controlCommand_pitch, 'p: ', pitch_controlCommand_p, 'd: ', pitch_controlCommand_d, 'i: ', pitch_controlCommand_i, 'pitch: ', attitude_msg.pitch

        print 'cc: ', self.controlCommand_roll, 'p: ', roll_controlCommand_p, 'd: ', roll_controlCommand_d, 'i: ', roll_controlCommand_i, 'roll: ', attitude_msg.roll

    def imuCallback(self, imu_msg):
        self.imu = imu_msg


if __name__ == '__main__':
    try:
        controller_node = AttitudeHoldNode()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass



                # if self.input_rc[4] > 1500:
#     self.setPoint = altitude_msg.estimated_altitude
#     self.controlCommand = self.input_rc[3]
#
# self.sum_i += self.setPoint - altitude_msg.estimated_altitude
# if self.sum_i >= self.i_stop:
#     self.sum_i = self.i_stop
# elif self.sum_i <= -self.i_stop:
#     self.sum_i = -self.i_stop
# controlCommand_p = (self.setPoint - altitude_msg.estimated_altitude) * self.p
# controlCommand_d = (self.setPoint_d - (altitude_msg.estimated_altitude - self.previousAltitude) * 100) * self.d
# controlCommand_i = self.sum_i * self.i
# un_cliped = self.controlCommand + controlCommand_p + controlCommand_d + controlCommand_i
# self.controlCommand = np.clip(un_cliped, 1000, 2000)
# joy_msg = Joy()
#
# # Replay and override current rc
# joy_msg.pitch = self.input_rc[0]
# joy_msg.roll = self.input_rc[1]
# joy_msg.yaw = self.input_rc[2]
# joy_msg.throttle = self.input_rc[3]
# joy_msg.aux1 = self.input_rc[4]
# joy_msg.aux2 = self.input_rc[5]
# joy_msg.aux3 = self.input_rc[6]
# joy_msg.aux4 = self.input_rc[7]
# self.previousAltitude = altitude_msg.estimated_altitude
# self.pub.publish(joy_msg)
# print 'set_point:', self.setPoint, '\t alt:', altitude_msg.estimated_altitude, '\t controlCommand', un_cliped, self.controlCommand, 'p:', controlCommand_p, 'i:', controlCommand_i, 'd:', controlCommand_d
#