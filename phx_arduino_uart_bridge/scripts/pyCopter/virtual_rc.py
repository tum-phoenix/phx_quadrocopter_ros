__author__ = 'manuelviermetz'


class virtual_remote_control:
    def __init__(self, copter):
        self.copter = copter
        """
            raw values:
            throttle           0  -   100
            pitch roll yaw  (-50) -   +50
            aux1 to aux4       0  -   100
        """
        self.throttle = 2
        self.pitch = 2
        self.roll = 3
        self.yaw = 3
        self.aux1 = 4
        self.aux2 = 5
        self.aux3 = 6
        self.aux4 = 7
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

    def update_rc(self, debug=False):
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