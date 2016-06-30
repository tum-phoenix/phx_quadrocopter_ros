import numpy as np

import rospy
from phx_uart_msp_bridge.msg import Diagnostics


class PIDController:

    def __init__(self, controlCommand, setPoint_p, p, d, i, setPoint_d, i_stop, i_mode, plot_slot):
        self.plot_slot = plot_slot
        self.set_point = setPoint_p
        self.i_mode = i_mode # with i mode to use, calculation of i value differs in different controller
        self.controlCommand = controlCommand
        self.previousAltitude = 0

        self.p = p
        self.d = d
        self.setPoint_d = setPoint_d
        self.i = i
        self.sum_i = 0
        self.i_stop = i_stop



        self.diag_pub = rospy.Publisher('/diag_out', Diagnostics, queue_size=1)

    def calculateControlCommand(self, current_p, current_d):

        if self.i_mode == 0:
            self.sum_i += self.set_point - current_p
        elif self.i_mode == 1: # used by take off controller
            if self.previousAltitude >= current_p:
                self.sum_i += 1
            else:
                self.sum_i = 0

        if self.sum_i >= self.i_stop:
            self.sum_i = self.i_stop
        elif self.sum_i <= -self.i_stop:
            self.sum_i = -self.i_stop
        controlCommand_p = (self.set_point - current_p) * self.p
        controlCommand_d = (self.setPoint_d - current_d) * self.d
        controlCommand_i = self.sum_i * self.i
        unclipped =  self.controlCommand + controlCommand_p + controlCommand_d + controlCommand_i

        # plot PID results
        plot = Diagnostics()
        plot.header.stamp.secs = rospy.get_time()
        if self.plot_slot == 0:
            plot.val_a0 = controlCommand_p
            plot.val_a1 = controlCommand_i
            plot.val_a2 = controlCommand_d
        if self.plot_slot == 1:
            plot.val_b0 = controlCommand_p
            plot.val_b1 = controlCommand_i
            plot.val_b2 = controlCommand_d
        if self.plot_slot == 2:
            plot.val_c0 = controlCommand_p
            plot.val_c1 = controlCommand_i
            plot.val_c2 = controlCommand_d

        if not (self.plot_slot < 0):
            self.diag_pub.publish(plot)


        self.previousAltitude = current_p # used by take off controller
        return unclipped


