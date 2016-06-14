class PIDController():
    def __init__(self, controlCommand, setPoint_p,p,d,i,setPoint_d, i_stop):
        self.set_point = setPoint_p

        self.controlCommand = controlCommand

        self.p = p
        self.d = d
        self.setPoint_d = setPoint_d
        self.i = i
        self.sum_i = 0
        self.i_stop = i_stop

    def calculateControlCommand(self, current_p, current_d):

        self.sum_i += self.set_point - current_p

        if self.sum_i >= self.i_stop:
            self.sum_i = self.i_stop
        elif self.sum_i <= -self.i_stop:
            self.sum_i = -self.i_stop
        controlCommand_p = (self.set_point - current_p) * self.p
        controlCommand_d = (self.setPoint_d - current_d) * self.d
        controlCommand_i = self.sum_i * self.i
        unclipped =  self.controlCommand + controlCommand_p + controlCommand_d + controlCommand_i
        return unclipped


