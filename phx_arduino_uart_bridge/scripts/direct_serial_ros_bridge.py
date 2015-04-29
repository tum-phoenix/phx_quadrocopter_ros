__author__ = 'manuelviermetz'

import serial
import time


class SerialCom:
    def __init__(self, serial_port, baudrate=115200):
        """
          multiwii standard baudrate is 115200
          marvic standard baudrate is 2000000

          the protocol follows the ideas of the MultiWii Serial Protocol
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

        self.rc = {'throttle': 0, 'pitch': 0, 'roll': 0, 'yaw': 0, 'aux1': 0, 'aux2': 0, 'aux3': 0, 'aux4': 0}
        self.sticks = [0, 0, 0, 0, 0, 0, 0, 0]
        self.motor = {'0': 99, '1': 99, '2': 98, '3': 99}
        self.battery = {'cell1': [1, 0, 0], 'cell2': [2, 0, 0], 'cell3': [3, 0, 0], 'cell4': [4, 0, 0]}
        self.status = {'cycleTime': 0}
        self.altitude = {'EstAlt': 0}
        self.attitude = {'pitch': 10, 'roll': 11, 'heading': 12}
        self.raw_imu = {'acc_0': 0, 'acc_1': 0, 'acc_2': 0,
                        'gyr_0': 0, 'gyr_1': 0, 'gyr_2': 0,
                        'mag_0': 0, 'mag_1': 0, 'mag_2': 0}
        self.gps = {'fix': 0, 'numSat': 0, 'coordLAT': 0, 'coordLON': 0, 'alt': 0, 'speed': 0, 'groundcourse': 0}
        
        self.startup_delay = 15.0
        self.time_of_last_receive = 0.0
        self.time_of_last_receive_timeout = 2.0
        self.startup_time = time.time()

        self.ser.open()
        self.connection_check()

        self.callback_imu = None
        self.callback_motor = None
        self.callback_battery = None
        self.callback_status = None
        self.callback_altitude = None
        self.callback_attitude = None
        self.callback_gps = None
        self.callback_rc = None

        self.request_rates = None

        self.message_statistic = {}
    
    def connection_check(self):
        """
            returns 1 if serial connection is OC else it returns 0 and reconnects the serial port.
        """
        if time.time() < self.startup_time + self.startup_delay:
            # in case we are just starting the connection we have to wait for some startup_time
            self.time_of_last_receive = time.time()
            return 0
        return 1       # this prevents any reconnecting during operation

    def receive(self, debug=False):
        if self.connection_check() == 1:
            if debug:
                print 'SERIAL_COM.receive in waiting', self.ser.inWaiting()
            while self.ser.inWaiting():
                start_byte = self.ser.read(1)
                if debug:
                    print 'SERIAL_COM.receive start_byte', start_byte
                if start_byte == "$":
                    while self.ser.inWaiting() < 4:
                        start_waiting = time.time()
                        time.sleep(0.00001)
                        if start_waiting < time.time() - 0.01:
                            print 'SERIAL_COM.receive did receive msg start but no data in time. no response since 0.01 sec A'
                            self.ser.flushInput()
                            return 0
                    reading = self.ser.read(4)
                    calc_checksum = 0
                    header = start_byte + reading[0:2]
                    length = ord(reading[2])
                    calc_checksum ^= length
                    code = ord(reading[3])
                    calc_checksum ^= code
                    start_waiting = time.time()
                    while self.ser.inWaiting() < length+1:
                        time.sleep(0.00001)
                        if start_waiting < time.time() - 0.01:
                            print 'SERIAL_COM.receive did receive msg start but no data in time. no response since 0.01 sec B'
                            self.ser.flushInput()
                            return 0
                    data = []
                    for i in range(0, length):
                        data.append(ord(self.ser.read(1)))
                        calc_checksum = calc_checksum ^ data[-1]
                    check_sum = ord(self.ser.read(1))
                    if debug:
                        print 'SERIAL_COM.receive > serial input:', header, length, code, data, check_sum, '<->', calc_checksum
                    self.time_of_last_receive = time.time()
                    if check_sum == calc_checksum:
                        if code in self.message_statistic.keys():
                            self.message_statistic[code][1] += 1
                        return self.interpret(code, data, header)
                else:
                    print 'SERIAL_COM.receive did receive non start byte at start position'
        else:
            if debug:
                print 'SERIAL_COM.receive connection check was bad'

    def send_msg(self, data=[], code=0, debug=False):
        """
            Sends a message following the multiwii serial protocol:
                $ M < %datalength %msg_code %%%data %checkbyte
            returns 1 if message was sent successfully else 0
        """
        checksum = 0
        data_length = len(data)
        total_data = bytearray([ord('$'), ord('M'), ord('<'), data_length, code] + data)
        checksum = checksum ^ data_length ^ code
        for d in data:
            checksum = checksum ^ d
        total_data.append(checksum)
        if debug:
            print ' >>> sending serial data', total_data
        if self.connection_check() == 0:
            return 0
        self.ser.write(total_data)
        return 1
    
    def send_request(self, cmd, debug=False):
        if self.connection_check() != 0:
            if cmd in self.message_statistic.keys():
                self.message_statistic[cmd][0] += 1
            else:
                self.message_statistic[cmd] = [1, 0]
        r = self.send_msg(code=cmd, debug=debug)
        if debug:
            print 'SERIAL_COM.send_request on command', cmd, 'sent', r

    def request(self, debug=False):
        if self.request_rates:
            for msg in self.request_rates.items():
                msg_code = msg[0]
                msg_frequency = msg[1][0]
                if msg_frequency != 0:
                    msg_next_request = msg[1][1]
                    if time.time() >= msg_next_request:
                        self.send_request(cmd=msg_code, debug=debug)
                        self.request_rates[msg_code][1] = time.time() + 1. / msg_frequency
                        self.receive(debug=debug)

    def get_msg(self, cmd_list=(66, 101, 102, 104, 105, 106, 108, 109), wait_for_answer=0.00001, debug=False):
        for cmd in cmd_list:
            self.send_request(cmd, debug=debug)
        if wait_for_answer:
            time.sleep(wait_for_answer)
            self.receive(debug=debug)
        if debug:
            print 'SERIAL_COM.get_msg done'
    
    def send_rc(self, sticks, debug=False):
        """
            sticks = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
            values between 1000 and 2000 as pwm.
        """
        serial_throttle, serial_pitch, serial_roll, serial_yaw, serial_aux1, serial_aux2, serial_aux3, serial_aux4 = sticks
        
        serial_msg_data = write_uint16(serial_roll) + write_uint16(serial_pitch) + write_uint16(serial_yaw) + write_uint16(serial_throttle) + write_uint16(serial_aux1) + write_uint16(serial_aux2) + write_uint16(serial_aux3) + write_uint16(serial_aux4)
        serial_msg_length = len(serial_msg_data)
        serial_msg_cmd = 200
        if debug:
            print 'SERIAL_COM.send_rc', serial_msg_cmd, serial_msg_length, [serial_roll, serial_pitch, serial_yaw, serial_throttle, serial_aux1, serial_aux2, serial_aux3, serial_aux4]
        self.send_msg(code=serial_msg_cmd, data=serial_msg_data)

    def send_via_ros_callback_rc(self, stuff):
        self.send_rc(sticks=[stuff.axes[0],
                             stuff.axes[1],
                             stuff.axes[2],
                             stuff.axes[3],
                             stuff.buttons[0],
                             stuff.buttons[1],
                             stuff.buttons[2],
                             stuff.buttons[3]])

    def send_motors(self, motors, debug=False):
        """
            motors = [ front_left, front_right, rear_right, rear_left ]
               values between 0 and 1.0 or pwm values between 1000 and 2000
        """
        if motors[0] < 500:
            serial_msg_data = write_uint16(1000 + motors[0] * 1000) + write_uint16(1000 + motors[1] * 1000) + write_uint16(1000 + motors[2] * 1000) + write_uint16(1000 + motors[3] * 1000)
        else:
            serial_msg_data = write_uint16(motors[0]) + write_uint16(motors[1]) + write_uint16(motors[2]) + write_uint16(motors[3])
        serial_msg_length = len(serial_msg_data)
        serial_msg_cmd = 214
        if debug:
            print 'SERIAL_COM.send_motors', serial_msg_cmd, serial_msg_length, [1000+motors[0]*1000, 1000+motors[1]*1000, 1000+motors[2]*1000, 1000+motors[3]*1000]
        self.send_msg(code=serial_msg_cmd, data=serial_msg_data)

    def send_via_ros_callback_motor(self, stuff):
        self.send_motors(motors=(stuff.motor0,
                                 stuff.motor1,
                                 stuff.motor2,
                                 stuff.motor3))
    
    def send_ip(self, ip_string, debug=False):
        """
            ip address as '192.168.0.2'
            msg_code = 64
        """
        try:
            ip_address = ip_string.split('.')
        except:
            print 'SERIAL_COM.send_ip >>> ip address was not in proper shape'
            return
        serial_msg_data = []
        if len(ip_address) == 4:
            for i in ip_address:
                serial_msg_data.append(int(i))
            serial_msg_length = len(serial_msg_data)
            serial_msg_cmd = 64
            if debug:
                print 'SERIAL_COM.send_ip', serial_msg_cmd, serial_msg_length, serial_msg_data
            self.send_msg(code=serial_msg_cmd, data=serial_msg_data, debug=debug)
        else:
            print 'SERIAL_COM.send_ip >>> ip address was too long ?!?!'

    def send_acc_calibration(self, debug=False):
        serial_msg_data = []
        serial_msg_length = 0
        serial_msg_cmd = 205

        if debug:
            print 'SERIAL_COM.send_acc_calibration', serial_msg_cmd, serial_msg_length, serial_msg_data
        self.send_msg(code=serial_msg_cmd)

    def interpret(self, code, data, header='$M>', debug=False):
        if header == '$M!':
            if debug:
                print 'SERIAL_COM.interpret: msg could not be interpreted', header, code, data
            return 1
        if code == 66:
            # BATTERY
            tuple_length = 6
            data.append(0)
            cell_number = 0
            for cell in range(0, len(data)/tuple_length):
                cell_number += 1
                cell_mean = read_uint16(data[cell*tuple_length:cell*tuple_length+2])
                cell_min = read_uint16(data[cell*tuple_length+2:cell*tuple_length+4])
                cell_max = read_uint16(data[cell*tuple_length+4:cell*tuple_length+6])
                self.battery['cell'+str(cell_number)] = [cell_mean, cell_min, cell_max]

            if self.callback_battery:
                self.callback_battery[0](topic=self.callback_battery[1],
                                         cell_voltages=(self.battery['cell1'][0],
                                                        self.battery['cell1'][1],
                                                        self.battery['cell1'][2],
                                                        self.battery['cell1'][3]))
            if debug:
                print 'battery updated',  self.battery
            return 1
        elif code == 101:
            # STATUS
            self.status['cycleTime'] = read_uint16(data[0:2])
            self.status['i2c_errors'] = read_uint16(data[2:4])
            self.status['sensor'] = read_uint16(data[4:6])
            self.status['flag'] = read_uint32(data[6:10])
            self.status['setting'] = read_uint8(data[10])

            if self.callback_status:
                self.callback_status[0](topic=self.callback_status[1],
                                        cycletime=self.status['cycleTime'])

            if debug:
                print 'status updated', self.status
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
            
            if self.callback_imu:
                self.callback_imu[0](topic=self.callback_imu[1],
                                     acc=(self.raw_imu['acc_0'],
                                          self.raw_imu['acc_1'],
                                          self.raw_imu['acc_2']),
                                     gyr=(self.raw_imu['gyr_0'],
                                          self.raw_imu['gyr_1'],
                                          self.raw_imu['gyr_2']),
                                     mag=(self.raw_imu['mag_0'],
                                          self.raw_imu['mag_1'],
                                          self.raw_imu['mag_2']),
                                     attitude=(self.attitude['pitch'],
                                               self.attitude['roll'],
                                               self.attitude['heading'],
                                               self.altitude['EstAlt']))

            if debug:
                print 'raw imu updated', self.raw_imu
        elif code == 104:
            # MSP_Motor
            self.motor['0'] = read_uint16(data[0:2])
            self.motor['1'] = read_uint16(data[2:4])
            self.motor['2'] = read_uint16(data[4:6])
            self.motor['3'] = read_uint16(data[6:])

            if self.callback_motor:
                self.callback_motor[0](topic=self.callback_motor[1],
                                       motor=(self.motor['0'],
                                              self.motor['1'],
                                              self.motor['2'],
                                              self.motor['3']))

            if debug:
                print 'motors updated', self.motor
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

            if self.callback_rc:
                self.callback_rc[0](topic=self.callback_rc[1],
                                    sticks=(self.rc['throttle'],
                                            self.rc['pitch'],
                                            self.rc['roll'],
                                            self.rc['yaw'],
                                            self.rc['aux1'],
                                            self.rc['aux2'],
                                            self.rc['aux3'],
                                            self.rc['aux4']))

            if debug:
                print 'rc updated', self.rc
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

            if self.callback_gps:
                self.callback_gps[0](topic=self.callback_gps[1],
                                     gps_lat=self.gps['coordLAT'],
                                     gps_lon=self.gps['coordLON'],
                                     gps_alt=self.gps['alt'])

            if debug:
                print 'gps updated', self.gps
            return 1
        elif code == 108:
            # ATTITUDE
            self.attitude['roll'] = read_int16(data[0:2])
            self.attitude['pitch'] = read_int16(data[2:4])
            self.attitude['heading'] = read_int16(data[4:])

            if self.callback_attitude:
                self.callback_attitude[0](topic=self.callback_attitude[1],
                                          acc=(self.raw_imu['acc_0'],
                                               self.raw_imu['acc_1'],
                                               self.raw_imu['acc_2']),
                                          gyr=(self.raw_imu['gyr_0'],
                                               self.raw_imu['gyr_1'],
                                               self.raw_imu['gyr_2']),
                                          mag=(self.raw_imu['mag_0'],
                                               self.raw_imu['mag_1'],
                                               self.raw_imu['mag_2']),
                                          attitude=(self.attitude['pitch'],
                                                    self.attitude['roll'],
                                                    self.attitude['heading'],
                                                    self.altitude['EstAlt']))

            if debug:
                print 'attitude updated', self.attitude
            return 1
        elif code == 109:
            # ALTITUDE
            self.altitude['EstAlt'] = 0.01 * read_int32(data[0:4])
            self.altitude['vario'] = 0.01 * read_int16(data[4:])

            if self.callback_altitude:
                self.callback_altitude[0](topic=self.callback_altitude[1],
                                          altitude=self.altitude['EstAlt'])

            if debug:
                print 'altitude updated', self.altitude
            return 1
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

import time
import numpy as np
import sys

import rospy
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from sensor_msgs.msg import FluidPressure #Barometer
from sensor_msgs.msg import Temperature #For compensation gyrodrift
from sensor_msgs.msg import Range #Distance to ground
from geometry_msgs.msg import Twist, Quaternion
from phx_arduino_uart_bridge.msg import Motor
from phx_arduino_uart_bridge.msg import Battery
from phx_arduino_uart_bridge.msg import Cycletime
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue #For Battery status


class RosCom():
    def __init__(self, node_name, update_rate=50, preset=None, callback_option=SerialCom):
        self.current_time_stamp = None
        self.next_time_stamp_update = time.time()
        self.time_stamp_precision = 0.01

        self.node_name = node_name
        rospy.init_node(self.node_name)

        self.update_rate = update_rate
        self.rate = rospy.Rate(self.update_rate)

        self.publishers = {}
        self.subscribers = {}

        if preset == 'MultiwiiSerial':
            #   create publisher
            # rc_multiwii           [pwm]
            # cycletime_multiwii    [microseconds]
            # imu_multiwii          [...]
            # gps                   [...]
            # motor_multiwii        [pwm]
            self.add_publisher(topic='/phoenix/imu_multiwii', msg_type='imu', queue_size=5)
            self.add_publisher(topic='/phoenix/cycletime_multiwii', msg_type='cycletime', queue_size=3)
            self.add_publisher(topic='/phoenix/rc_multiwii', msg_type='joy', queue_size=3)
            self.add_publisher(topic='/phoenix/gps_multiwii', msg_type='gps', queue_size=3)
            self.add_publisher(topic='/phoenix/motor_status_multiwii', msg_type='motor', queue_size=3)

            if callback_option:
                callback_option.callback_attitude = [self.publish_imu, '/phoenix/imu_multiwii']
                callback_option.callback_status = [self.publish_cycletime, '/phoenix/cycletime_multiwii']
                callback_option.callback_rc = [self.publish_joy, '/phoenix/rc_multiwii']
                callback_option.callback_gps = [self.publish_gps, '/phoenix/gps_multiwii']
                callback_option.callback_motor = [self.publish_motor, '/phoenix/motor_status_multiwii']

            #   create subscriber
            # motor_multiwii        [pwm]
            if callback_option:
                callback = callback_option.send_via_ros_callback_rc
            else:
                callback = None
            self.add_subscriber(topic='/phoenix/motor_multiwii', msg_type='motor', callback_function=callback)

        elif preset == 'MarvicSerial':
            #   create publisher
            # rc_ground             [pwm]
            # cycletime_marvic      [microseconds]
            # imu_marvic            [...]
            # gps                   [...]
            # battery               [milli volt]
            self.add_publisher(topic='/phoenix/imu_marvic', msg_type='imu', queue_size=5)
            self.add_publisher(topic='/phoenix/cycletime_marvic', msg_type='cycletime', queue_size=1)
            self.add_publisher(topic='/phoenix/rc_ground', msg_type='joy', queue_size=1)
            self.add_publisher(topic='/phoenix/gps_marvic', msg_type='gps', queue_size=1)
            self.add_publisher(topic='/phoenix/battery', msg_type='battery', queue_size=1)

            if callback_option:
                callback_option.callback_imu = [self.publish_imu, '/phoenix/imu_marvic']
                callback_option.callback_status = [self.publish_cycletime, '/phoenix/cycletime_marvic']
                callback_option.callback_rc = [self.publish_joy, '/phoenix/rc_ground']
                callback_option.callback_gps = [self.publish_gps, '/phoenix/gps_marvic']
                callback_option.callback_battery = [self.publish_battery, '/phoenix/battery']

            #   create subscriber
            # rc_computer           [pwm]
            # motor_marvic          [pwm]
            # local_ip              not implemented
            # sound                 not implemented
            # light                 not implemented
            # camera_move           not implemented
            if callback_option:
                callback = callback_option.send_via_ros_callback_rc
            else:
                callback = None
            self.add_subscriber(topic='/phoenix/rc_computer', msg_type='joy', callback_function=callback)
            if callback_option:
                callback = callback_option.send_via_ros_callback_motor
            else:
                callback = None
            self.add_subscriber(topic='/phoenix/motor_marvic', msg_type='motor', callback_function=callback)

    def add_publisher(self, topic='/phoenix/stat_imu', msg_type=None, queue_size=5):
        if topic not in self.publishers.keys():
            if msg_type == 'imu':
                publisher = rospy.Publisher(topic, Imu, queue_size=queue_size)
                message = Imu()
            elif msg_type == 'motor':
                publisher = rospy.Publisher(topic, Motor, queue_size=queue_size)
                message = Motor()
            elif msg_type == 'gps':
                publisher = rospy.Publisher(topic, NavSatFix, queue_size=queue_size)
                message = NavSatFix()
            elif msg_type == 'joy':
                publisher = rospy.Publisher(topic, Joy, queue_size=queue_size)
                message = Joy()
            elif msg_type == 'cycletime':
                publisher = rospy.Publisher(topic, Cycletime, queue_size=queue_size)
                message = Cycletime()
            elif msg_type == 'battery':
                publisher = rospy.Publisher(topic, Battery, queue_size=queue_size)
                message = Battery()
            else:
                print 'ROSCOM.add_publisher this msg_type is not defined:', msg_type, 'on topic', topic
                return False
            self.publishers[topic] = [publisher, message]
        else:
            print 'ROSCOM.add_publisher this topic is already published:', topic
        return True

    def add_subscriber(self, topic='/phoenix/stat_imu', msg_type=None, callback_function=None):
        if not callback_function:
            callback_function = self.default_callback
        if topic not in self.subscribers.keys():
            if msg_type == 'imu':
                subscriber = rospy.Subscriber(topic, Imu, callback_function)
            elif msg_type == 'motor':
                subscriber = rospy.Subscriber(topic, Motor, callback_function)
            elif msg_type == 'gps':
                subscriber = rospy.Subscriber(topic, NavSatFix, callback_function)
            elif msg_type == 'joy':
                subscriber = rospy.Subscriber(topic, Joy, callback_function)
            elif msg_type == 'cycletime':
                subscriber = rospy.Subscriber(topic, Cycletime, callback_function)
            elif msg_type == 'battery':
                subscriber = rospy.Subscriber(topic, Battery, callback_function)
            else:
                print 'ROSCOM.add_subscriber this msg_type is not defined:', msg_type, 'on topic', topic
                return False
            self.subscribers[topic] = [subscriber, callback_function]
        else:
            print 'ROSCOM.add_subscriber this topic is already subscribed:', topic
        return True

    def default_callback(self, stuff):
        print 'ROSCOM.default_callback:', stuff

    def listen(self):
        self.rate.sleep()

    def is_shutdown(self):
        return rospy.is_shutdown()

    def update_time_stamp(self, overwrite=False):
        if time.time() > self.next_time_stamp_update or overwrite:
            self.next_time_stamp_update = time.time() + self.time_stamp_precision
            self.current_time_stamp = rospy.get_rostime()

    def publish_imu(self, topic, acc, gyr, mag, attitude, debug=False):
        """
        This publishes an imu message into the ros network on the given topic.
        :param topic: string
            a valid topic which was added via add_publisher
        :param acc: list or tuple
            (accX, accY, accZ)
        :param gyr: list or tuple
            (gyrX, gyrY, gyrZ)
        :param mag: list or tuple
            (magX, magY. magZ)
        :param attitude: list or tuple
            (pitch, roll, heading, altitude)
        :param debug: boolean
            some output will be generated
        :return:
            will return true in case everything was published
        """
        if topic in self.publishers.keys():
            publisher, message = self.publishers[topic]

            self.update_time_stamp()
            try:
                message.header.stamp.secs = self.current_time_stamp.secs
                message.header.stamp.nsecs = self.current_time_stamp.nsecs

                message.angular_velocity.x = -gyr[0]
                message.angular_velocity.y = -gyr[1]
                message.angular_velocity.z = -gyr[2]

                message.linear_acceleration.x = acc[0]
                message.linear_acceleration.y = acc[1]
                message.linear_acceleration.z = acc[2]

                q = tf.transformations.quaternion_from_euler(np.pi / 1800. * attitude[1],
                                                             np.pi / 1800. * attitude[0],
                                                             np.pi / 1800. * attitude[2])
                message.orientation = Quaternion(*q)

                publisher.publish(message)
                if debug:
                    print 'ROSCOM.publish_imu published imu: acc', acc, 'gyr', gyr, 'orientation', q
                return True
            except:
                etype, evalue, etb = sys.exc_info()
                print 'ROSCOM.publish_imu error:', etype, evalue, etb
        return False

    def publish_motor(self, topic, motor, debug=False):
        """
        This publishes a motor message into the ros network on the given topic.
        :param topic: string
            a valid topic which was added via add_publisher
        :param motor: list or tuple, (front_left, front_right, rear_right, rear_left)
            this is in pwm, meaning numbers from 1000 to 2000
        :param debug: boolean
            some output will be generated
        :return:
            will return true in case everything was published
        """
        if topic in self.publishers.keys():
            publisher, message = self.publishers[topic]

            self.update_time_stamp()
            try:
                #message.header.stamp.secs = self.current_time_stamp.secs
                #message.header.stamp.nsecs = self.current_time_stamp.nsecs

                message.motor0 = motor[0]
                message.motor1 = motor[1]
                message.motor2 = motor[2]
                message.motor2 = motor[2]

                publisher.publish(message)
                if debug:
                    print 'ROSCOM.publish_motor published motor:', motor
                return True
            except:
                etype, evalue, etb = sys.exc_info()
                print 'ROSCOM.publish_motor error:', etype, evalue, etb
        return False

    def publish_gps(self, topic, gps_lat, gps_lon, gps_alt, debug=False):
        """
        This publishes a gps message into the ros network on the given topic.
        :param topic: string
            a valid topic which was added via add_publisher
        :param gps_lat: float

        :param gps_lon: float

        :param gps_alt: float

        :param debug: boolean
            some output will be generated
        :return:
            will return true in case everything was published
        """
        if topic in self.publishers.keys():
            publisher, message = self.publishers[topic]

            self.update_time_stamp()
            try:
                message.header.stamp.secs = self.current_time_stamp.secs
                message.header.stamp.nsecs = self.current_time_stamp.nsecs

                message.latitude = gps_lat
                message.longitude = gps_lon
                message.altitude = gps_alt

                publisher.publish(message)
                if debug:
                    print 'ROSCOM.publish_gps published gps:', gps_lat, gps_lon, gps_alt
                return True
            except:
                etype, evalue, etb = sys.exc_info()
                print 'ROSCOM.publish_gps error:', etype, evalue, etb
        return False

    def publish_battery(self, topic, cell_voltages, debug=False):
        """
        This publishes a battery message into the ros network on the given topic.
        :param topic: string
            a valid topic which was added via add_publisher
        :param cell_voltages: list or tuple

        :param debug: boolean
            some output will be generated
        :return:
            will return true in case everything was published
        """
        if topic in self.publishers.keys():
            publisher, message = self.publishers[topic]

            self.update_time_stamp()
            try:
                #message.header.stamp.secs = self.current_time_stamp.secs
                #message.header.stamp.nsecs = self.current_time_stamp.nsecs

                message.cell1 = cell_voltages[0]
                message.cell2 = cell_voltages[1]
                message.cell3 = cell_voltages[2]
                message.cell4 = cell_voltages[3]

                publisher.publish(message)
                if debug:
                    print 'ROSCOM.publish_battery published cell voltages:', cell_voltages
                return True
            except:
                etype, evalue, etb = sys.exc_info()
                print 'ROSCOM.publish_battery error:', etype, evalue, etb
        return False

    def publish_cycletime(self, topic, cycletime, debug=False):
        """
        This publishes a cycletime message into the ros network on the given topic.
        :param topic: string
            a valid topic which was added via add_publisher
        :param cycletime: float or integer

        :param debug: boolean
            some output will be generated
        :return:
            will return true in case everything was published
        """
        if topic in self.publishers.keys():
            publisher, message = self.publishers[topic]

            self.update_time_stamp()
            try:
                #message.header.stamp.secs = self.current_time_stamp.secs
                #message.header.stamp.nsecs = self.current_time_stamp.nsecs

                message.cycletime = cycletime

                publisher.publish(message)
                if debug:
                    print 'ROSCOM.publish_cycletime published cycletime:', cycletime
                return True
            except:
                etype, evalue, etb = sys.exc_info()
                print 'ROSCOM.publish_cycletime error:', etype, evalue, etb
        return False

    def publish_joy(self, topic, sticks, debug=False):
        """
        This publishes a joy message into the ros network on the given topic.
        :param topic: string
            a valid topic which was added via add_publisher
        :param sticks: tuple or list, [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
            this is in pwm, meaning numbers from 1000 to 2000
        :param debug: boolean
            some output will be generated
        :return:
            will return true in case everything was published
        """
        if topic in self.publishers.keys():
            publisher, message = self.publishers[topic]

            self.update_time_stamp()
            try:
                message.header.stamp.secs = self.current_time_stamp.secs
                message.header.stamp.nsecs = self.current_time_stamp.nsecs

                message.axes = sticks[:4]
                message.buttons = sticks[4:]

                publisher.publish(message)
                if debug:
                    print 'ROSCOM.publish_cycletime published cycletime:', cycletime
                return True
            except:
                etype, evalue, etb = sys.exc_info()
                print 'ROSCOM.publish_cycletime error:', etype, evalue, etb
        return False