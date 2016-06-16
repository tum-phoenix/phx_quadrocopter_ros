import numpy as np
class PIDController():
    def __init__(self, controlCommand, setPoint_p,p,d,i,setPoint_d, i_stop, i_mode):
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

    def calculateControlCommand(self, current_p, current_d):

        if(self.i_mode == 0):
            self.sum_i += self.set_point - current_p
        elif self.i_mode == 1: # used by take off controller
            if (self.previousAltitude >= current_p):
                self.i_sum += 1
            else:
                self.i_sum = 0

        if self.sum_i >= self.i_stop:
            self.sum_i = self.i_stop
        elif self.sum_i <= -self.i_stop:
            self.sum_i = -self.i_stop
        controlCommand_p = (self.set_point - current_p) * self.p
        controlCommand_d = (self.setPoint_d - current_d) * self.d
        controlCommand_i = self.sum_i * self.i
        unclipped =  self.controlCommand + controlCommand_p + controlCommand_d + controlCommand_i
        self.controlCommand = np.clip(unclipped,1000,2000)

        self.previousAltitude = current_p # used by take off controller
        return self.controlCommand


