__author__ = 'manuelviermetz'

import ros_com
import serial_com
import network_com
import time
import numpy as np
import speed


class copter:
    def __init__(self, con_multiwii=True, con_intermediate=True, con_ros=False, osc_transmit=(None, 10000), osc_receive=10001):
        """
            con_multiwii: True/False            serial connection to the multiwii flight controller board
            con_intermediate: True/False        serial connection to the intermediate arduino rc controller
            osc_transmit: [string, int]/False   [ ip address, port ] of receiving destination to monitor the status of the copter
            osc_receive: int/False              port on which the copter is listening for osc_rc input e.g. mobile gyro
            
            to ensure proper start call check_startup() after init.
        """
        # connect to serial multiwii
        if con_multiwii:
            self.serial_multiwii = serial_com.multiwii_protocol('/dev/multiwii', 115200)
            self.serial_multiwii.startup_delay = 15.0
        else:
            self.serial_multiwii = None
        self.serial_multiwii_request_counter = 0

        # connect to serial intermediate
        if con_intermediate:
            self.serial_intermediate = serial_com.multiwii_protocol('/dev/marvic', 115200)
            self.serial_intermediate.startup_delay = 10.
        else:
            self.serial_intermediate = None

        # start ros node
        if con_ros:
            self.ros_node = ros_com.ros_communication(copter=self)
        else:
            self.ros_node = None

        # define some timing variables
        self.interval_read_serial = 1./70.
        self.interval_send_serial_low_priority = 10.0   # IP, position_LED
        self.interval_send_serial_rc = 1./40.
        self.interval_send_osc_status = 1./15.
        self.interval_update_status = 1./50.
        self.interval_update_ros = 1./50.
        self.timer_read_serial = 0
        self.timer_send_serial_low_priority = 0
        self.timer_send_serial_rc = 0
        self.timer_send_osc_status = 0
        self.timer_update_status = 0
        self.timer_update_ros = 0

        # define the copter status variables
        self.imu_acc = [0, 0, 0]
        self.imu_gyr = [0, 0, 0]
        self.imu_mag = [0, 0, 0]
        self.attitude = [0, 0, 0, 0]
        self.motors = [0, 0, 0, 0]
        self.gps = [0, 0, 0, 0, 0]    # [lat, lon, alt, speed, course]
        self.cycletime_0 = [0]
        self.cycletime_1 = [0]
        self.rc0 = [0, 0, 0, 0, 0, 0, 0, 0]     # original RC signals from Human operator
        self.rc1 = [0, 0, 0, 0, 0, 0, 0, 0]     # virtual RC which was generated by the computer
        self.rc2 = [0, 0, 0, 0, 0, 0, 0, 0]     # RC which is actually received by the multiwii
        self.battery = [0, 0, 0, 0]
        self.position_led = [0]

        self.last_sent_motor_values = [10, 11, 12, 13]

        self.local_ip = network_com.get_local_ip()

        # define default rc for serial out
        self.use_rc = None

        # start an OSC transmitter for the status data
        if osc_transmit:
            self.status_transmitter = network_com.OSC_transmitter(destination=osc_transmit[0], port=osc_transmit[1])
        else:
            self.status_transmitter = None

        # start an OSC receiver for OSC_rc input
        if osc_receive:
            self.osc_remote = network_com.OSC_rc()
            self.copter_receiver = network_com.OSC_receiver(osc_rc=self.osc_remote, osc_status_transmitter=self.status_transmitter, port=osc_receive)
            self.copter_receiver.start()
        else:
            self.osc_remote = None
            self.copter_receiver = None
        self.speed_1 = speed.speedtest()
        self.speed_2 = speed.speedtest()
        self.speed_3 = speed.speedtest()
        self.speed_4 = speed.speedtest()
        self.speed_5 = speed.speedtest()
        self.speed_6 = speed.speedtest()

    def stop(self):
        """
            this stops the osc server ... important, since otherwise the process can not be terminated cleanly
        """
        if self.copter_receiver:
            self.copter_receiver.stop()
        if self.status_transmitter:
            self.status_transmitter.stop()
        print ' >>> Copter stopped!'

    def update(self, debug=False):
        """
            This updates all variables of copter from their source and to their destination.
            The timings are defined in the copters init section about timings.
            --> This can not be called too often since it handles its rate on it's own.
        """
        self.speed_1.start()
        self.serial_receive_update(debug=debug)         # receives data on serial connections...ensures that we receive data and the buffer does not overflow
        self.speed_1.stop()

        self.speed_2.start()
        self.update_status(debug=debug)                 # sends requests and receives the answers if they are replied instantly
        self.speed_2.stop()

        self.speed_3.start()
        if self.serial_intermediate:
            self.send_serial_rc(debug=debug)            # sends the default RC command to the intermediate arduino (default RC is set by self.use_rc)
            self.send_serial_low_priority(debug=debug)  # sends serial messages - low rate
        self.speed_3.stop()

        self.speed_4.start()
        if self.status_transmitter:
            self.send_osc_status(debug=debug)           # publishes status to OSC listeners
        self.speed_4.stop()
        
        self.speed_5.start()
        if self.ros_node:
            self.update_ros()
        self.speed_5.stop()
        
        self.speed_6.start()
        self.serial_receive_update(debug=debug)         # receives data on serial connections...ensures that we receive data and the buffer does not overflow
        self.speed_6.stop()
        
        self.speed_1.print_result(rate=1., text=' > copterstatus speed_1 takes')
        self.speed_2.print_result(rate=1., text=' > copterstatus speed_2 takes')
        self.speed_3.print_result(rate=1., text=' > copterstatus speed_3 takes')
        self.speed_4.print_result(rate=1., text=' > copterstatus speed_4 takes')
        self.speed_5.print_result(rate=1., text=' > copterstatus speed_5 takes')
        self.speed_6.print_result(rate=1., text=' > copterstatus speed_6 takes')

    def update_ros(self):
        if self.timer_update_ros < time.time():
            self.time_update_ros = time.time() + self.interval_update_ros
            self.ros_node.listen()
            self.ros_node.update_time_stamp()
            if self.serial_multiwii:
                self.ros_node.pub_motors(self.motors)
                self.ros_node.pub_imu(acc=self.imu_acc, gyr=self.imu_gyr, mag=self.imu_mag, attitude=self.attitude)
                self.ros_node.pub_rc2(self.rc2)
                self.ros_node.pub_gps(gps_lat=self.gps[0], gps_lon=self.gps[1], gps_alt=self.gps[2])
                self.ros_node.pub_cycletime0(self.cycletime_0[0])
            if self.serial_intermediate:
                self.ros_node.pub_battery(self.battery)
                self.ros_node.pub_rc0(self.rc0)
                self.ros_node.pub_rc1(self.rc1)
                self.ros_node.pub_cycletime1(self.cycletime_1[0])
#            self.ros_node.listen()

    def serial_receive_update(self, debug=False):
        """
            this needs to be called very regularly to avoid buffer overflow and connection loss.
        """
        if self.serial_multiwii:
            self.serial_multiwii.receive(debug=debug)
        if self.serial_intermediate:
            self.serial_intermediate.receive(debug=debug)

    def update_status(self, remote_control=None, debug=False):
        if self.timer_update_status < time.time():
            self.timer_update_status = time.time() + self.interval_update_status

            # update from multiwii
            if self.serial_multiwii:
                if self.serial_multiwii_request_counter == 0:                               # get rc2 and attitude
                    self.serial_multiwii.get_msg(cmd_list=[105, 108], debug=debug)
                    self.serial_multiwii_request_counter = 1
                elif self.serial_multiwii_request_counter == 1:                             # get imu, rc2 and attitude
                    self.serial_multiwii.get_msg(cmd_list=[102, 105, 108], debug=debug)
                    self.serial_multiwii_request_counter = 2
                elif self.serial_multiwii_request_counter == 2:                             # get rc2 and attitude
                    self.serial_multiwii.get_msg(cmd_list=[105, 108], debug=debug)
                    self.serial_multiwii_request_counter = 3
                elif self.serial_multiwii_request_counter == 3:                             # get imu, rc2 and status
                    self.serial_multiwii.get_msg(cmd_list=[102, 105, 101], debug=debug)
                    self.serial_multiwii_request_counter = 4
                elif self.serial_multiwii_request_counter == 4:                             # get imu, rc2 and motor
                    self.serial_multiwii.get_msg(cmd_list=[102, 105, 104], debug=debug)
                    self.serial_multiwii_request_counter = 5
                elif self.serial_multiwii_request_counter == 5:                             # get imu, rc2 and gps
                    self.serial_multiwii.get_msg(cmd_list=[102, 105, 106], debug=debug)
                    self.serial_multiwii_request_counter = 6
                elif self.serial_multiwii_request_counter == 7:                             # get imu, rc2 and altitude
                    self.serial_multiwii.get_msg(cmd_list=[102, 105, 109], debug=debug)
                    self.serial_multiwii_request_counter = 0
                else:
                    self.serial_multiwii_request_counter = 0
                #self.serial_multiwii.get_msg(cmd_list=[101, 102, 104, 105, 106, 108, 109], debug=debug)
                self.imu_acc = [self.serial_multiwii.raw_imu['acc_0'], self.serial_multiwii.raw_imu['acc_1'], self.serial_multiwii.raw_imu['acc_2']]
                self.imu_gyr = [self.serial_multiwii.raw_imu['gyr_0'],  self.serial_multiwii.raw_imu['gyr_1'], self.serial_multiwii.raw_imu['gyr_2']]
                self.imu_mag = [self.serial_multiwii.raw_imu['mag_0'], self.serial_multiwii.raw_imu['mag_1'], self.serial_multiwii.raw_imu['mag_2']]
                self.attitude = [self.serial_multiwii.attitude['pitch'], self.serial_multiwii.attitude['roll'], self.serial_multiwii.attitude['heading'],
                                 self.serial_multiwii.altitude['EstAlt']]
                self.motors = [self.serial_multiwii.motor['0'], self.serial_multiwii.motor['1'], self.serial_multiwii.motor['2'],
                               self.serial_multiwii.motor['3']]
                self.gps = [self.serial_multiwii.gps['coordLAT'], self.serial_multiwii.gps['coordLON'], self.serial_multiwii.gps['alt'],
                            self.serial_multiwii.gps['speed'], self.serial_multiwii.gps['groundcourse']]
                self.cycletime_0 = [self.serial_multiwii.status['cycleTime']]
                self.rc2 = [self.serial_multiwii.rc['throttle'], self.serial_multiwii.rc['pitch'], self.serial_multiwii.rc['roll'],
                            self.serial_multiwii.rc['yaw'], self.serial_multiwii.rc['aux1'], self.serial_multiwii.rc['aux2'], self.serial_multiwii.rc['aux3'],
                            self.serial_multiwii.rc['aux4']]

            # update from intermediate
            if self.serial_intermediate:
                self.serial_intermediate.get_msg(cmd_list=[66, 101, 102, 105], debug=debug)
                self.battery = [self.serial_intermediate.battery['cell1'][0], self.serial_intermediate.battery['cell2'][0],
                                self.serial_intermediate.battery['cell3'][0], self.serial_intermediate.battery['cell4'][0]]
                self.cycletime_1 = [self.serial_intermediate.status['cycleTime']]
                self.rc0 = [self.serial_intermediate.rc['throttle'], self.serial_intermediate.rc['pitch'], self.serial_intermediate.rc['roll'],
                            self.serial_intermediate.rc['yaw'], self.serial_intermediate.rc['aux1'], self.serial_intermediate.rc['aux2'],
                            self.serial_intermediate.rc['aux3'], self.serial_intermediate.rc['aux4']]

            # update from local
            if remote_control:
                self.rc1 = remote_control.get_pwm_sticks()
            elif self.use_rc:
                self.rc1 = self.use_rc.get_pwm_sticks()
            else:
                self.rc1 = [1001, 1001, 1001, 1001, 1001, 1001, 1001, 1001]

    def send_osc_status(self, debug=False):
        """
            this sends the current data to the monitoringPC.
        """
        if self.status_transmitter and self.timer_send_osc_status < time.time():
            self.timer_send_osc_status = time.time() + self.interval_send_osc_status
            if self.serial_multiwii:
                self.status_transmitter.send_imu(self.imu_acc + self.imu_gyr + self.imu_mag + self.attitude, debug=debug)
                self.status_transmitter.send_motors(self.motors)
                self.status_transmitter.send_cycletime0(self.cycletime_0)
                self.status_transmitter.send_rc2(self.rc2)
            if self.serial_intermediate:
                self.status_transmitter.send_cycletime1(self.cycletime_1)
                self.status_transmitter.send_rc0(self.rc0)
                self.status_transmitter.send_rc1(self.rc1)
                self.status_transmitter.send_battery(self.battery)

    def send_serial_rc(self, remote_control=None, debug=False):
        """
            This updates the RC commands to the intermediate Arduino.
            In case a remote_control is set, this remote is used, otherwise the one in self.use_rc is used.
            If none of these is defined the fallback is the original rc0.
        """
        if self.serial_intermediate and self.timer_send_serial_rc < time.time():
            self.timer_send_serial_rc = time.time() + self.interval_send_serial_rc
            if debug: print ' >>> copter send_serial_rc sending:'
            if remote_control:
                self.serial_intermediate.send_rc(sticks=remote_control.get_pwm_sticks(), pwm=True, debug=debug)
            elif self.use_rc:
                if debug: print '      via use_rc defined rc is used'
                self.serial_intermediate.send_rc(sticks=self.use_rc.get_pwm_sticks(), pwm=True, debug=debug)
            else:
                self.serial_intermediate.send_rc(sticks=self.rc0, pwm=True, debug=debug)

    def send_serial_low_priority(self, debug=False):
        if self.serial_intermediate and self.timer_send_serial_low_priority < time.time():
            self.timer_send_serial_low_priority = time.time() + self.interval_send_serial_low_priority
            # send local ip to intermediate
            self.serial_intermediate.send_ip(self.local_ip, debug=debug)
            # send position light to intermediate
            self.serial_intermediate.send_position_light(self.position_led, debug=debug)

    def send_serial_motor(self, motor_values=(1050, 1050, 1050, 1050), debug=False):
        self.last_sent_motor_values = motor_values
        print 'this function send_serial_motor() is not jet implemented.', motor_values
