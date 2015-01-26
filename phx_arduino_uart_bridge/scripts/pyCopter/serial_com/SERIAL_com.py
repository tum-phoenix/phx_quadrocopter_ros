__author__ = 'manuelviermetz'

import serial
import time


class multiwii_protocol:
    def __init__(self, serial_port, baudrate=115200):
        """
          multiwii standard baudrate is 115200
          
          see http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
        """
        self.ser = serial.Serial()
        self.ser.port = serial_port
        self.ser.baudrate = baudrate
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2
        
        self.startup_delay = 10.0
        self.time_of_last_receive = 0.0
        self.time_of_last_receive_timeout = 2.0
        self.startup_time = 0.0
        
        self.rc = {'throttle': 0, 'pitch': 0, 'roll': 0, 'yaw': 0, 'aux1': 0, 'aux2': 0, 'aux3': 0, 'aux4': 0}
        self.sticks = [0, 0, 0, 0, 0, 0, 0, 0]
        self.motor = {'0': 99, '1': 99, '2': 98, '3': 99}
        self.battery = {'cell0': [1, 0, 0], 'cell1': [2, 0, 0], 'cell2': [3, 0, 0], 'cell3': [4, 0, 0], 'cell4': [5, 0, 0]}
        self.status = {'cycleTime': 0}
        self.altitude = {'EstAlt': 0}
        self.attitude = {'pitch': 10, 'roll': 11, 'heading': 12}
        self.raw_imu = {'acc_0': 0, 'acc_1': 0, 'acc_2': 0,
                        'gyr_0': 0, 'gyr_1': 0, 'gyr_2': 0,
                        'mag_0': 0, 'mag_1': 0, 'mag_2': 0}
        self.gps = {'fix': 0, 'numSat': 0, 'coordLAT': 0, 'coordLON': 0, 'alt': 0, 'speed': 0, 'groundcourse': 0}
        self.connection_check()
    
    def connection_check(self):
        """
            returns 1 if serial connection is OC else it returns 0 and reconnects the serial port.
        """
        if time.time() < self.startup_time + self.startup_delay:
            # in case we are just starting the connection we have to wait for some startup_time
            self.time_of_last_receive = time.time()
            return 0
        if time.time() > self.time_of_last_receive + self.time_of_last_receive_timeout:
            # no contact since a long time! let's try a reconnect
            try:
                print 'reconnecting ', self.ser.getPort()
                self.ser.open()
                self.ser.flushOutput()
                self.ser.flushInput()
                self.time_of_last_receive = time.time()
                self.startup_time = time.time()
                print 'reconnected!', self.ser.getPort(), 'startup_time is set to', self.startup_time, 'seconds'
            except:
                print '>>> serial port', self.ser.getPort(), 'down...probably disconnected! next check in', self.time_of_last_receive_timeout, 'seconds'
                self.ser.close()
                print '>>> closed', self.ser.getPort()
                self.time_of_last_receive = time.time()
        if self.ser.isOpen():
            # everything is fine
            return 1
        else:
            # connection is down...after self.time_of_last_contact_timeout seconds a reconnect will be initiated
            return 0
    
    def check_startup(self, debug=True):
        """
            this is only for debugging, do not use this in any program code. Only for command line!
        """
        if time.time() < self.startup_time + self.startup_delay:
            print 'doing startup sleep'
            time.sleep(self.startup_time + self.startup_delay - time.time())
        if debug: print 'receiving all msgs'
        self.receive()
        if debug: print 'doing first update'
        self.get_msg(debug=debug)
        if debug: print 'receiving all msgs'
        self.receive()
        if debug: print ' >>> serial_com properly started on', self.ser.port
        self.time_of_last_receive_timeout = 600.0
        print 'changed time_of_last_receive_timeout to 600.0, this turns the reconnection on time out off!'

    def receive(self, debug=False):
        # a wide try except is necessary since it can happen that the connection is broken while receiving.
        # in this case the script would break.
        try:
            if self.connection_check() == 1:
                while self.ser.inWaiting() > 5:
                    start_byte = self.ser.read(1)
                    if debug: print 'start_byte', start_byte
                    if start_byte == "$":
                        calc_checksum = 0
                        header = start_byte + self.ser.read(2)
                        length = ord(self.ser.read(1))
                        calc_checksum ^= length
                        code = ord(self.ser.read())
                        calc_checksum ^= code
                        start_waiting = time.time()
                        while self.ser.inWaiting() < length+1:
                            if start_waiting < time.time() - 0.1:
                                self.ser.flushInput()
                                return 0
                        data = []
                        for i in range(0, length):
                            data.append(ord(self.ser.read()))
                            calc_checksum = calc_checksum ^ data[-1]
                        check_sum = ord(self.ser.read())
                        if debug: print ' > serial input:', header, length, code, data, check_sum, '<->', calc_checksum,
                        self.time_of_last_receive = time.time()
                        if check_sum == calc_checksum:
                            if debug: print '-> valid msg, doing interpretation'
                            try:
                                return self.interpret(code, data, header)
                            except:
                                print ' >>> error in interpretation of msg', header, code, length, data, check_sum, ' <-> ', calc_checksum
                                return 0
                        if debug: print '\n\n'
        except:
            try:
                self.ser.close()
                print '>>> closed', self.ser.getPort(), 'because of error in receive loop. connection_check will clean this up.'
            except:
                pass

    def receive_loop(self, debug=False):
        """
            Infinite receiving loop! Only for command line use!
        """
        while True:
            self.receive(debug=debug)

    def send_msg(self, data=(), code=0, data_length=None, debug=False):
        """
            Sends a message following the multiwii serial protocol:
                $ M < %datalength %msg_code %%%data %checkbyte
            returns 1 if message was sent successfully else 0
        """
        checksum = 0
        if not data_length:
            data_length = len(data)
        total_data = [ord('$'), ord('M'), ord('<'), data_length, code] + data
        checksum = checksum ^ data_length ^ code
        for d in data:
            checksum = checksum ^ d
        total_data.append(checksum)
        if debug: print ' >>> sending serial data', total_data
        bytes_sent = 0
        if self.connection_check() == 0:
            return 0
        try:
            for byte in total_data:
                a = self.ser.write(chr(byte))
                bytes_sent += a
        except Exception, error:
            return 0
        if bytes_sent != len(total_data):
            print 'not everything sent!'
        self.ser.flush()
        return 1
    
    def send_request(self, cmd, debug=False):
        self.send_msg(data_length=0, code=cmd, data=[])
        if debug: print 'cmd', cmd, 'sent'
        if debug: print 'sending done'

    def get_msg(self, cmd_list=(66, 101, 102, 104, 105, 106, 108, 109), debug=False):
        for cmd in cmd_list:
            self.send_request(cmd, debug=debug)
        self.receive(debug=debug)
        if debug: print ' >>> get_msg done'
    
    def send_rc(self, sticks, pwm=False, debug=False):
        """
            sticks = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
               throttle           0  -   100
               pitch roll yaw  (-50) -   +50 
               aux1 to aux4       0  -   100
            or values between 1000 and 2000 if pwm flag is True
            The best is to use pwm_sticks here! even if it is not the default!
        """
        if not pwm:
            throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 = sticks
            serial_throttle = 1000 + 10. * throttle
            serial_pitch = 1500 + 10. * pitch
            serial_roll = 1500 + 10. * roll
            serial_yaw = 1500 + 10. * yaw
            serial_aux1 = 1000 + 10. * aux1
            serial_aux2 = 1000 + 10. * aux2
            serial_aux3 = 1000 + 10. * aux3
            serial_aux4 = 1000 + 10. * aux4
        else:
            serial_throttle, serial_pitch, serial_roll, serial_yaw, serial_aux1, serial_aux2, serial_aux3, serial_aux4 = sticks
        
        serial_msg_data = write_uint16(serial_roll) + write_uint16(serial_pitch) + write_uint16(serial_yaw) + write_uint16(serial_throttle) + write_uint16(serial_aux1) + write_uint16(serial_aux2) + write_uint16(serial_aux3) + write_uint16(serial_aux4)
        serial_msg_length = len(serial_msg_data)
        serial_msg_cmd = 200
        if debug: print 'sending rc', serial_msg_cmd, serial_msg_length, [serial_roll, serial_pitch, serial_yaw, serial_throttle, serial_aux1, serial_aux2, serial_aux3, serial_aux4]
        self.send_msg(code=serial_msg_cmd, data=serial_msg_data)

    def send_flight_directions(self, directions, debug=False):
        """
            msg_code = 68
            directions = [ flight_direction, flight_speed, altitude_change ]
            all in int16_t
        """
        if len(directions) >= 3:
            serial_msg_data = write_int16(directions[0]) + write_int16(directions[1]) + write_int16(directions[2])
            serial_msg_length = len(serial_msg_data)
            serial_msg_cmd = 68
            if debug: print 'sending directions', serial_msg_cmd, serial_msg_length, directions
            self.send_msg(code=serial_msg_cmd, data=serial_msg_data)
        else:
            print ' >>> directions list does not fit, length needs to be three'
    
    def send_position_light(self, mode, debug=False):
        """
            msg_code = 67
            mode = [ mode_number ]
            important: a list has to be provided!
        """
        serial_msg_data = [mode[0]]
        serial_msg_length = 1
        serial_msg_cmd = 67
        
        if debug: print 'sending position light', serial_msg_cmd, serial_msg_length, serial_msg_data
        self.send_msg(code=serial_msg_cmd, data=serial_msg_data)
    
    def send_ip(self, ip_string, debug=False):
        """
            msg_code = 64
        """
        try:
            ip_address = ip_string.split('.')
        except:
            print ' >>> ip address was not in proper shape'
            return
        serial_msg_data = []
        if len(ip_address) == 4:
            for i in ip_address:
                serial_msg_data.append(int(i))
            serial_msg_length = len(serial_msg_data)
            serial_msg_cmd = 64
            if debug: print 'sending ip address', serial_msg_cmd, serial_msg_length, serial_msg_data
            self.send_msg(code=serial_msg_cmd, data=serial_msg_data, debug=debug)
        else:
            print ' >>> ip address was too long ?!?!'

    def send_acc_calibration(self, debug=False):
        serial_msg_data = []
        serial_msg_length = 0
        serial_msg_cmd = 205

        if debug: print 'sending acc calibration', serial_msg_cmd, serial_msg_length, serial_msg_data
        self.send_msg(code=serial_msg_cmd, data=serial_msg_data)

    def interpret(self, code, data, header='$M>', debug=False):
        if header == '$M!':
            if debug: print 'msg could not be interpreted', header, code, data
            return 1
        if code == 66:
            # BATTERY - intermediate
            tupel_length = 7
            data.append(0)
            for cell in range( 0, len(data)/tupel_length ):
                cell_number = data[cell*tupel_length+0]
                cell_mean = read_uint16(data[cell*tupel_length+1:cell*tupel_length+3])
                cell_min = read_uint16(data[cell*tupel_length+3:cell*tupel_length+5])
                cell_max = read_uint16(data[cell*tupel_length+5:cell*tupel_length+7])
                self.battery['cell'+str(cell_number)] = [cell_mean, cell_min, cell_max]
            if debug: print 'battery updated',  self.battery
            return 1
        elif code == 101:
            # STATUS
            self.status['cycleTime'] = read_uint16(data[0:2])
            self.status['i2c_errors'] = read_uint16(data[2:4])
            self.status['sensor'] = read_uint16(data[4:6])
            self.status['flag'] = read_uint32(data[6:10])
            self.status['setting'] = read_uint8(data[10])
            if debug: print 'status updated', self.status
            return 1
        elif code == 102:
            # RAW_IMU
            self.raw_imu['acc_0'] = read_int16(data[0:2])
            self.raw_imu['acc_1'] = read_int16(data[2:4])
            self.raw_imu['acc_2'] = read_int16(data[4:6])
            self.raw_imu['gyr_0'] = read_int16(data[6:8])
            self.raw_imu['gyr_1'] = read_int16(data[8:10])
            self.raw_imu['gyr_2'] = read_int16(data[10:12])
            self.raw_imu['mag_0'] = read_int16(data[12:14])
            self.raw_imu['mag_1'] = read_int16(data[14:16])
            self.raw_imu['mag_2'] = read_int16(data[16:])
            if debug: print 'raw imu updated', self.raw_imu
        elif code == 104:
            # MSP_Motor
            self.motor['0'] = read_uint16(data[0:2])
            self.motor['1'] = read_uint16(data[2:4])
            self.motor['2'] = read_uint16(data[4:6])
            self.motor['3'] = read_uint16(data[6:])
            if debug: print 'motors updated', self.motor
            return 1
        elif code == 105:
            # MSP_RC
            self.rc['roll'] = read_uint16(data[0:2])
            self.rc['pitch'] = read_uint16(data[2:4])
            self.rc['yaw'] = read_uint16(data[4:6])
            self.rc['throttle'] = read_uint16(data[6:8])
            self.rc['aux1'] = read_uint16(data[8:10])
            self.rc['aux2'] = read_uint16(data[10:12])
            self.rc['aux3'] = read_uint16(data[12:14])
            self.rc['aux4'] = read_uint16(data[14:])
            if debug: print 'rc updated', self.rc
            return 1
        elif code == 106:
            # RAW_GPS
            self.gps['fix'] = read_uint8(data[0])
            self.gps['numSat'] = read_uint8(data[1])
            self.gps['coordLAT'] = read_uint32(data[2:6])
            self.gps['coordLON'] = read_uint32(data[6:10])
            self.gps['alt'] = read_uint16(data[10:12])
            self.gps['speed'] = read_uint16(data[12:14])
            self.gps['groundcourse'] = read_uint16(data[14:])
            if debug: print 'gps updated', self.gps
            return 1
        elif code == 108:
            # ATTITUDE
            self.attitude['roll'] = read_int16(data[0:2])
            self.attitude['pitch'] = read_int16(data[2:4])
            self.attitude['heading'] = read_int16(data[4:])
            if debug: print 'attitude updated', self.attitude
            return 1
        elif code == 109:
            # ALTITUDE
            self.altitude['EstAlt'] = 0.01* read_int32(data[0:4])
            self.altitude['vario'] = 0.01 * read_int16(data[4:])
            if debug: print 'altitude updated', self.altitude
            return 1
        return 0

    def get_pwm_sticks(self):
        """
            returns the pwm stick positions on a scale between 1000 and 1500
        """
        return [self.rc['throttle'], self.rc['pitch'], self.rc['roll'], self.rc['yaw'], self.rc['aux1'], self.rc['aux2'], self.rc['aux3'], self.rc['aux4']]

    def get_pwm_stick(self, name):
        """
            returns one pwm stick position on a scale between 1000 and 1500
        """
        if name in ['throttle', 'pitch', 'roll', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']:
            return self.rc[name]
        else:
            return 1500

    def get_sticks(self):
        """
            returns the raw stick positions on a scale between 0 and 100 or -50 to 50
        """
        self.sticks = [(self.rc['throttle']-1000)/10.,
                       (self.rc['pitch']-1500)/10.,
                       (self.rc['roll']-1500)/10.,
                       (self.rc['yaw']-1500)/10.,
                       (self.rc['aux1']-1000)/10.,
                       (self.rc['aux2']-1000)/10.,
                       (self.rc['aux3']-1000)/10.,
                       (self.rc['aux4']-1000)/10.]
        return self.sticks

    def get_stick(self, name):
        """
            returns one raw stick position on a scale between 0 and 100 or -50 to 50
        """
        if name in ['throttle', 'aux1', 'aux2', 'aux3', 'aux4']:
            return (self.rc[name]-1000)/10.
        elif name in ['pitch', 'roll', 'yaw']:
            return (self.rc[name]-1500)/10.
        else:
            print ' >>> multiwii_protocol get_stick(', name, ') request could not be answered correctly since this stick name is not available'
            return 50.

    def set_sticks(self, sticks, debug=False):
        print ' >>> multiwii_protocol is a read-only rc!'
        if debug:
            print ' >>> the sticks were not set:', sticks
        return 0

    def set_stick(self, name, value, debug=False):
        print ' >>> multiwii_protocol is a read-only rc!'
        if debug:
            print ' >>> the stick was not set:', name, value
        return 0


def write_uint16(data):
    data1 = int(data) % 255
    data0 = int(data) / 255
    out = [data0, data1]
    return out


def write_int16(data):
    if data < 0:
        data = data + 65536 - 255
    data1 = int(data) % 255
    data0 = int(data) / 255
    out = [data0, data1]
    return out


def read_uint8(data):
    data_0 = int(data)
    out = data_0
    return out


def read_uint16(data):
    data_0 = int(data[0])
    data_1 = int(data[1])
    out = 255 * data_1 + data_0
    return out


def read_int16(data):
    data_0 = int(data[0])
    data_1 = int(data[1])
    out = 255 * data_1 + data_0
    if out > 65536 / 2:
        out = out - 65536 + 255
    return out


def read_uint32(data):
    data_0 = int(data[0])
    data_1 = int(data[1])
    data_2 = int(data[2])
    data_3 = int(data[3])
    out = 16777216 * data_3 + 65536 * data_2 + 255 * data_1 + data_0
    return out


def read_int32(data):
    data_0 = int(data[0])
    data_1 = int(data[1])
    data_2 = int(data[2])
    data_3 = int(data[3])
    out = 16777216 * data_3 + 65536 * data_2 + 255 * data_1 + data_0
    if out > 2147483648:
        out = out - 4294967296 + 255
    return out