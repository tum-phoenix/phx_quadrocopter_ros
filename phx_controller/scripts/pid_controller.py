import numpy as np

import rospy
from phx_uart_msp_bridge.msg import Diagnostics


class PIDController:

    def __init__(self, control_command, p, i, d, set_point_p, i_stop, set_point_d, i_mode):
        self.set_point = set_point_p
        self.i_mode = i_mode # with i mode to use, calculation of i value differs in different controller
        self.control_command = control_command
        self.previous_altitude = 0

        self.p = p
        self.d = d
        self.set_point_d = set_point_d
        self.i = i
        self.sum_i = 0
        self.i_stop = i_stop



        self.diag_pub = rospy.Publisher('/diag_out', Diagnostics, queue_size=1)

    def calculate_control_command(self, current_p, current_d):

        if self.i_mode == 0:
            self.sum_i += self.set_point - current_p
        elif self.i_mode == 1: # used by take off controller
            if self.previous_altitude >= current_p:
                self.sum_i += 1
            else:
                self.sum_i = 0

        if self.sum_i >= self.i_stop:
            self.sum_i = self.i_stop
        elif self.sum_i <= -self.i_stop:
            self.sum_i = -self.i_stop
        control_command_p = (self.set_point - current_p) * self.p
        control_command_d = (self.set_point_d - current_d) * self.d
        control_command_i = self.sum_i * self.i
        unclipped = self.control_command + control_command_p + control_command_d + control_command_i

        # plot PID results
        # plotting only works correctly when only one controller is active
        plot = Diagnostics()
        plot.header.stamp.secs = rospy.get_time()
        #if self.plot_slot == 0:
        plot.val_a0 = control_command_p
        plot.val_a1 = control_command_i
        plot.val_a2 = control_command_d
        '''
        if self.plot_slot == 1:
            plot.val_b0 = control_command_p
            plot.val_b1 = control_command_i
            plot.val_b2 = control_command_d
        if self.plot_slot == 2:
            plot.val_c0 = control_command_p
            plot.val_c1 = control_command_i
            plot.val_c2 = control_command_d
        '''

        self.diag_pub.publish(plot)


        self.previous_altitude = current_p # used by take off controller

        print 'PIDController sagt: control_command:', self.control_command, 'p:', control_command_p, 'i:', control_command_i, 'd:', control_command_d
        return unclipped


