import numpy as np
import rospy
from pid_controller import PIDController
from phx_uart_msp_bridge.msg import Attitude
from phx_uart_msp_bridge.msg import AutoPilotCmd
from phx_uart_msp_bridge.msg import RemoteControl
from phx_uart_msp_bridge.msg import ControllerCmd
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu


class AttitudeHoldNode():
    def __init__(self):
        rospy.init_node('attitude_hold_controller')
        self.node_identifier = 2
        self.input_rc = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.sub_imu = rospy.Subscriber('/phx/imu', Imu, self.imuCallback)
        self.sub_attitude = rospy.Subscriber('/phx/fc/attitude', Attitude, self.attitudeCallback)
        self.autopilot_commands = rospy.Subscriber('/phx/controller_commands', ControllerCmd, self.controllerCommandCallback)
        self.pub = rospy.Publisher('/phx/autopilot/input', AutoPilotCmd, queue_size=1)

        self.imu = Imu()

        self.freq = 100  # Hz
        self.r = rospy.Rate(self.freq)
        self.enabled = True

        self.currentPose = Attitude()

        self.yawController = PIDController(1500, 1, 0, 1, 0, 100, 0, 0)
        self.pitchController = PIDController(1500, 1, 0, 1, 0, 100, 0, 0)
        self.rollController = PIDController(1500, 1, 0, 1, 0, 100, 0, 0)


        self.controlCommand_pitch = 1500
        self.controlCommand_roll = 1500
        self.controlCommand_yaw = 1500

    def run(self):
        while not rospy.is_shutdown():
            self.r.sleep()

    def attitudeCallback(self, attitude_msg):
        if self.enabled:
            self.currentPose = attitude_msg
            self.currentPose.yaw -= 180

            controlCommand_pitch = self.pitchController.calculate_control_command(attitude_msg.pitch, self.imu.angular_velocity.y)

            controlCommand_roll = self.rollController.calculate_control_command(attitude_msg.roll, self.imu.angular_velocity.x)

            controlCommand_yaw = self.yawController.calculate_control_command(attitude_msg.yaw, self.imu.angular_velocity.z)

            autopilot_command = AutoPilotCmd()

            joy_msg = RemoteControl()
            joy_msg.pitch = controlCommand_pitch
            joy_msg.roll = controlCommand_roll
            joy_msg.yaw = controlCommand_yaw
            autopilot_command.rc = joy_msg
            autopilot_command.node_identifier = self.node_identifier

            self.pub.publish(autopilot_command)

            #print 'cc: ', self.controlCommand_pitch, 'p: ', pitch_controlCommand_p, 'd: ', pitch_controlCommand_d, 'i: ', pitch_controlCommand_i, 'pitch: ', attitude_msg.pitch
            #print 'cc: ', self.controlCommand_roll, 'p: ', roll_controlCommand_p, 'd: ', roll_controlCommand_d, 'i: ', roll_controlCommand_i, 'roll: ', attitude_msg.roll
            #print 'x: ', self.imu.angular_velocity.x, 'y: ', self.imu.angular_velocity.y, 'z: ', self.imu.angular_velocity.z
            print controlCommand_yaw, controlCommand_roll, controlCommand_pitch
            #print 'cc: ', self.controlCommand_roll, 'p: ', roll_controlCommand_p, 'd: ', yaw_controlCommand_d, 'i: ', yaw_controlCommand_i, 'yaw: ', attitude_msg.yaw


    def imuCallback(self, imu_msg):
        self.imu = imu_msg

    def controllerCommandCallback(self, controller_msg):
        self.enabled = controller_msg.enabled[self.node_identifier]


if __name__ == '__main__':
    try:
        controller_node = AttitudeHoldNode()
        controller_node.enabled = True
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
