__author__ = 'manuelviermetz'

import time


class OSC_rc:
    def __init__(self):
        self.simple_directions = [0, 0, 0, 0]   # backward-forward, left-right, up-down, left-right-turn
        self.simple_directions_linear = [10, 10, 10, 10]

        self.gyro = [0, 0, 0]
        self.acc = [0, 0, 0]
        self.gps = []
        self.last_receive = 0
        self.printout = False
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

    def calc_rc_from_gyr(self, throttle=None, debug=False):
        """
            setting the raw values
        """
        if not throttle:
            self.throttle = self.throttle
        else:
            self.throttle = throttle

        self.pitch = (-1.)*self.gyro[0]*7.5 + 0.03*((-1.)*self.gyro[0]*10)**3
        if abs(self.pitch) > 25:
            self.pitch = 25.0 * (self.pitch/abs(self.pitch))

        self.roll = self.gyro[1]*7.5 + 0.03*(self.gyro[1]*10)**3
        if abs(self.roll) > 25:
            self.roll = 25.0 * (self.roll/abs(self.roll))

        self.yaw = (-1.)*self.gyro[2]*20
        if abs(self.yaw) < 10:
            self.yaw = 0
        elif abs(self.yaw) > 35:
            self.yaw = 30

        if debug: print ' >>> new OSC rc:', self.throttle, self.pitch, self.roll, self.yaw
        if debug: print ' >>> last OSC receive:', time.time()-self.last_receive

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

        if debug: print ' >>> new OSC rc:', self.throttle, self.pitch, self.roll, self.yaw
        if debug: print ' >>> last OSC receive:', time.time()-self.last_receive

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
        self.pwm_pitch    = self.stick_curve(self.pitch, mid_point=self.pwm_midpoint_pitch, mode=0)
        self.pwm_roll     = self.stick_curve(self.roll, mid_point=self.pwm_midpoint_roll, mode=0)
        self.pwm_yaw      = self.stick_curve(self.yaw, mid_point=self.pwm_midpoint_yaw, mode=0)
        self.pwm_aux1     = self.stick_curve(self.aux1, mid_point=self.pwm_midpoint_aux1, mode=0)
        self.pwm_aux2     = self.stick_curve(self.aux2, mid_point=self.pwm_midpoint_aux2, mode=0)
        self.pwm_aux3     = self.stick_curve(self.aux3, mid_point=self.pwm_midpoint_aux3, mode=0)
        self.pwm_aux4     = self.stick_curve(self.aux4, mid_point=self.pwm_midpoint_aux4, mode=0)
        self.validate_pwm()
        if debug: print 'after: raw ', self.throttle, self.pitch, self.roll, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4
        if debug: print 'after: pwm ', self.pwm_throttle, self.pwm_pitch, self.pwm_roll, self.pwm_yaw, self.pwm_aux1, self.pwm_aux2, self.pwm_aux3, self.pwm_aux4

    def get_sticks(self):
        return [self.throttle, self.pitch, self.roll, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4]

    def get_pwm_sticks(self, update_from_raw_sticks=True):
        if update_from_raw_sticks:
            self.calc_pwm_from_raw()
        return [self.pwm_throttle, self.pwm_pitch, self.pwm_roll, self.pwm_yaw, self.pwm_aux1, self.pwm_aux2, self.pwm_aux3, self.pwm_aux4]

    def set_sticks(self, sticks, debug=False):
        print ' >>> OSC_rc is a read-only rc!'
        if debug:
            print ' >>> the sticks were not set:', sticks
        return 0

    def set_stick(self, name, value, debug=False):
        print ' >>> osc_rc is a read-only rc!'
        if debug:
            print ' >>> the stick was not set:', name, value
        return 0

    def handle_gyro(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/gyro fff [0.48758959770202637, 0.06476165354251862, -0.19856473803520203] ('192.168.0.33', 57527)
        self.gyro = stuff
        self.last_receive = time.time()
        if self.printout:
            print ' >>> OSC_rc', self.gyro, ' gyro'

    def handle_acc(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/accel fff [0.48758959770202637, 0.06476165354251862, -0.19856473803520203] ('192.168.0.3$
        self.acc = stuff
        self.last_receive = time.time()
        if self.printout:
            print ' >>> OSC_rc', self.acc, ' accel'

    def handle_gps(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/gps fff
        self.gps = stuff
        self.last_receive = time.time()
        if self.printout:
            print ' >>> OSC_rc', self.gps, ' gps'

    def handle_ip(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/ipport fff
        self.ip = stuff
        self.last_receive = time.time()
        if self.printout:
            print ' >>> OSC_rc', self.ip, ' ip port'

    def handle_directions(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/ipport fff
        print 'received directions:', stuff, 'this is printed by OSC_remote_control.handle_directions()'
        self.simple_directions = stuff
        self.last_receive = time.time()
        if self.printout:
            print ' >>> OSC_rc', self.ip, ' directions'

    def handle_commands(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/ipport fff
        print 'received commands:', stuff
        self.last_receive = time.time()
        if self.printout:
            print ' >>> OSC_rc', self.ip, ' commands'
