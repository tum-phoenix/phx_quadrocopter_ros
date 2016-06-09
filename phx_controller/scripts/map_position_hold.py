#!/usr/bin/env python
import numpy as np
import tf
import tf2_ros
import rospy
from phx_uart_msp_bridge.msg import RemoteControl, Diagnostics
from sensor_msgs.msg import Joy


class GPSHoldNode():
    def __init__(self):
        rospy.init_node('map_position_hold')
        self.rate = 10
        self.r = rospy.Rate(self.rate)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.target_pos = np.array([1, 1, 0])
        self.copter_pos = np.zeros(3)
        self.copter_rot = np.zeros(3)

        # PID parameters
        self.error = 0
        self.p_gain = 1
        self.i_gain = 1
        self.i_sum = 0
        self.i_limit = 100
        self.d_gain = 1
        self.estimated_velocity = 0

        self.rc_input = RemoteControl()
        self.rc_input.pitch = 1500
        self.rc_input.roll = 1500
        self.rc_input.yaw = 1500
        self.rc_input.throttle = 1500
        self.rc_input.aux1 = 1000
        self.rc_input.aux2 = 1000
        self.rc_input.aux3 = 1000
        self.rc_input.aux4 = 1000

        self.rc_sub = rospy.Subscriber('/phx/rc_marvic', Joy, self.rc_callback)
        self.cmd_pub = rospy.Publisher('/phx/rc_computer', RemoteControl, queue_size=1)
        self.diag_pub = rospy.Publisher('/diagnostics', Diagnostics, queue_size=1)

    def get_cur_pos(self):
        try:
            trans = self.tfBuffer.lookup_transform('map', 'footprint', rospy.Time())
            self.copter_pos[0] = trans.transform.translation.x
            self.copter_pos[1] = trans.transform.translation.y
            self.copter_pos[2] = trans.transform.translation.z
            quaternion = (trans.transform.rotation.x,
                          trans.transform.rotation.y,
                          trans.transform.rotation.z,
                          trans.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.copter_rot[0] = euler[0]   # pitch
            self.copter_rot[1] = euler[1]   # roll
            self.copter_rot[2] = euler[2]   # yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        return self.copter_pos

    def rc_callback(self,rc_msg):
        self.rc_input[0] = rc_msg.axes[0]
        self.rc_input[1] = rc_msg.axes[1]
        self.rc_input[2] = rc_msg.axes[2]
        self.rc_input[3] = rc_msg.axes[3]
        self.rc_input[4] = rc_msg.buttons[0]
        self.rc_input[5] = rc_msg.buttons[1]
        self.rc_input[6] = rc_msg.buttons[2]
        self.rc_input[7] = rc_msg.buttons[3]

    def run(self):
        while not rospy.is_shutdown():
            controller_node.get_cur_pos()
            print '\n--------------------\ncurrent position:', controller_node.copter_pos
            print 'current rotation:', controller_node.copter_rot

            # calculate PID parameters
            # distance to target
            target_vector = np.array([self.copter_pos[0] - self.target_pos[0], self.copter_pos[1] - self.target_pos[1]])
            previous_error = self.error
            # maybe use more than one previous error, check if rospy rate higher than tf rate (de/dt will be 0)
            self.error = np.linalg.norm(target_vector)
            self.i_sum += self.error
            if self.i_sum >= self.i_limit:
                self.i_sum = self.i_limit
            elif self.i_sum <= -self.i_limit:
                self.i_sum = -self.i_limit

            if not previous_error == self.error:
                self.estimated_velocity = (previous_error - self.error) * self.rate

            control_d = self.estimated_velocity * self.d_gain
            control_p = self.error * self.p_gain
            control_i = self.i_sum * self.i_gain

            pid_result = control_p + control_i + control_d
            print 'pid result: ', pid_result

            # calculate angle to target
            angle = np.arctan(target_vector[1]/target_vector[0]) + self.copter_rot[2]

            rotation_z = np.array([[np.cos(angle), np.sin(angle), 0],
                                [-np.sin(angle), np.cos(angle), 0],
                                [0, 0, 1]])

            # determine ratio of pitch & roll, when the angle is 0 (copter points to target), roll = 0.
            ratio = np.array([1, 0, 0])
            ratio = rotation_z.dot(ratio)

            # convert to pitch/roll commands with scaling factor for the PID controller
            lower_speed, upper_speed = -1*pid_result , 1*pid_result
            ratio = np.interp(ratio,[-1, 1],[lower_speed,upper_speed])
            print 'ratio: ', ratio

            # override current rc
            rc_message = self.rc_input
            rc_message.pitch = 1500 + ratio[0]
            rc_message.roll = 1500 + ratio [1]
            # clip results
            np.clip(rc_message.pitch, 1000, 2000)
            np.clip(rc_message.roll, 1000, 2000)
            # use altitude_hold_command
            # rc_message.throttle = self.controlCommand_throttle
            self.cmd_pub.publish(rc_message)

            # plot PID results
            plot = Diagnostics()
            plot.header.stamp.secs = rospy.get_time()
            plot.val_a0 = control_p
            plot.val_a1 = control_i
            plot.val_a2 = control_d
            self.diag_pub.publish(plot)

            self.r.sleep()


if __name__ == '__main__':
    try:
        controller_node = GPSHoldNode()
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
