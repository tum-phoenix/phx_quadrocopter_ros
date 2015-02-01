__author__ = 'manuelviermetz'

import os
import time
import OSC
import threading
from OSC_remote_control import *
import sys
import socket
import numpy as np


class OSC_receiver:
    def __init__(self, osc_rc=None, osc_base_station=None, osc_status_transmitter=None, port=10000):
        # connecting modules to this osc receiver
        self.osc_rc = osc_rc
        self.osc_base_station = osc_base_station
        self.osc_status_transmitter = osc_status_transmitter

        # get own ip address
        self.local_ip = get_local_ip()
        print 'local ip', self.local_ip

        # set ip addresses
        self.receive_address = self.local_ip, port

        # OSC - CONNECT
        self.osc_receiver = OSC.OSCServer(self.receive_address)         # making this machine a osc receiving server
        self.thread = None                                              # will be started soon

        # connect handlers
        self.osc_receiver.addDefaultHandlers()
        # connecting handlers to functions
        if self.osc_rc:
            self.osc_receiver.addMsgHandler("/gyrosc/gyro"      , self.osc_rc.handle_gyro)      # from gyrosc App
            self.osc_receiver.addMsgHandler("/gyrosc/accel"     , self.osc_rc.handle_acc)       # from gyrosc App
            self.osc_receiver.addMsgHandler("/gyrosc/ipport"    , self.osc_rc.handle_ip)        # from gyrosc App
            self.osc_receiver.addMsgHandler("/gyrosc/gps"       , self.osc_rc.handle_gps)       # from gyrosc App
            print ' osc_receiver attached osc_rc handlers'
        if self.osc_base_station:
            self.osc_receiver.addMsgHandler("/status/time",  self.osc_base_station.handle_status_time)
            self.osc_receiver.addMsgHandler("/status/rc0",    self.osc_base_station.handle_status_rc0)
            self.osc_receiver.addMsgHandler("/status/rc1",    self.osc_base_station.handle_status_rc1)
            self.osc_receiver.addMsgHandler("/status/rc2",    self.osc_base_station.handle_status_rc2)
            self.osc_receiver.addMsgHandler("/status/imu",   self.osc_base_station.handle_status_imu)
            self.osc_receiver.addMsgHandler("/status/cycletime0",    self.osc_base_station.handle_status_cycletime0)
            self.osc_receiver.addMsgHandler("/status/cycletime1",   self.osc_base_station.handle_status_cycletime1)
            self.osc_receiver.addMsgHandler("/status/motors",   self.osc_base_station.handle_status_motors)
            print ' osc_receiver attached osc_base_station handlers'
        if self.osc_status_transmitter:
            self.osc_receiver.addMsgHandler("/osc_base_station/connect",  self.osc_status_transmitter.handle_connect)
            self.osc_receiver.addMsgHandler("/osc_base_station/directions",  self.osc_rc.handle_directions)
            self.osc_receiver.addMsgHandler("/osc_base_station/commands",  self.osc_rc.handle_commands)
    
    def start(self):
        print " >>> Starting OSCServer. Use ctrl-C to quit."
        self.thread = threading.Thread(target=self.osc_receiver.serve_forever)
        self.thread.start()
        print ' >>> OSC_receiver started on', self.receive_address, '\n'
    
    def stop(self):
        self.osc_receiver.close()
        self.thread.join()
        print ' >>> OSC_receiver closed'


class OSC_transmitter:
    def __init__(self, destination=None, port=10000, printout=False, ip_change=True):
        if destination and port:
            self.destination_ip = destination
            self.destination_port = port
        else:
            self.destination_ip = get_local_ip()
            self.destination_port = port

        self.ip_change = ip_change
        self.printout = printout
        self.destination_address = self.destination_ip, self.destination_port

        self.osc_transmitter = OSC.OSCClient()
        self.osc_transmitter.connect(self.destination_address)

    def stop(self):
        self.osc_transmitter.close()
        print ' >>> OSC_transmitter closed'

    def handle_connect(self, add, tag, stuff, source):
        source_ip = source[0]
        if source_ip != self.destination_ip and self.ip_change:
            self.osc_transmitter.close()
            self.destination_address = source_ip, self.destination_port
            self.osc_transmitter.connect(self.destination_address)
            print ' >>> new osc transmitter connection established', self.destination_address

    def send_connect(self, ip='0.0.0.0', debug=False):
        try:
            self.osc_transmitter.send(OSC.OSCMessage('/osc_base_station/connect', ip))
            if debug: print ' >>> sent connection request'
        except OSC.OSCClientError:
            if self.printout: print ' >>> sent connection -> but nobody is listening'

    def send_imu(self, imu=(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13), debug=False):
        """
            imu = [ accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY. magZ, pitch, roll, heading, altitude ]
        """
        if len(imu) == 13:
            try:
                self.osc_transmitter.send(OSC.OSCMessage('/status/imu', imu))
                if debug: print ' >>> sent imu'
            except OSC.OSCClientError:
                if self.printout: print ' >>> sent imu -> but nobody is listening'
        else:
            print ' >>> imu array too short -> nothing sent'
    
    def send_rc0(self, rc0=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
            rc0 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        if len(rc0) == 8:
            try:
                self.osc_transmitter.send(OSC.OSCMessage('/status/rc0', rc0))
                if debug: print ' >>> sent rc0'
            except OSC.OSCClientError:
                if self.printout: ' >>> sent rc0 -> but nobody is listening'
        else:
            print ' >>> rc0 array too short -> nothing sent'
    
    def send_rc1(self, rc1=(1, 2, 3, 4, 5, 6, 7, 8), debug=False):
        """
            rc1 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        if len(rc1) == 8:
            try:
                self.osc_transmitter.send(OSC.OSCMessage('/status/rc1', rc1))
                if debug: print ' >>> sent rc1'
            except OSC.OSCClientError:
                if self.printout: print ' >>> sent rc1 -> but nobody is listening'
        else:
            print ' >>> rc1 array too short -> nothing sent'
    
    def send_rc2(self, rc2=(10, 20, 30, 40, 50, 60, 70, 80), debug=False):
        """
            rc2 = [ throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4 ]
        """
        if len(rc2) == 8:
            try:
                self.osc_transmitter.send(OSC.OSCMessage('/status/rc2', rc2))
                if debug: print ' >>> sent rc2'
            except OSC.OSCClientError:
                if self.printout: print ' >>> sent rc2 -> but nobody is listening'
        else:
            print ' >>> rc2 array too short -> nothing sent'
    
    def send_cycletime0(self, cycletime0=1, debug=False):
        """
            cycletime0 = 0
        """
        try:
            self.osc_transmitter.send( OSC.OSCMessage('/status/cycletime0', [cycletime0]))
            if debug: print ' >>> sent cycletime0'
        except OSC.OSCClientError:
            if self.printout: print ' >>> sent cycletime0 -> but nobody is listening'
    
    def send_cycletime1(self, cycletime1=2, debug=False):
        """
            cycletime1 = 0
        """
        try:
            self.osc_transmitter.send(OSC.OSCMessage('/status/cycletime1', [cycletime1]))
            if debug: print ' >>> sent cycletime1'
        except OSC.OSCClientError:
            if self.printout: print ' >>> sent cycletime1 -> but nobody is listening'

    def send_cycletime2(self, cycletime2=2, debug=False):
        """
            cycletime2 = 0
        """
        try:
            self.osc_transmitter.send(OSC.OSCMessage('/status/cycletime2', [cycletime2]))
            if debug: print ' >>> sent cycletime2'
        except OSC.OSCClientError:
            if self.printout: print ' >>> sent cycletime2 -> but nobody is listening'
    
    def send_motors(self, motors=(1, 2, 3, 4), debug=False):
        """
            motors = [ motor0, motor1, motor2, motor3 ]
        """
        if len(motors) == 4:
            try:
                self.osc_transmitter.send(OSC.OSCMessage('/status/motors', motors))
                if debug: print ' >>> sent motors'
            except OSC.OSCClientError:
                if self.printout: print ' >>> sent motors -> but nobody is listening'
        else:
            print ' >>> motors array too short -> nothing sent'
    
    def send_battery(self, battery=(1, 2, 3, 0), debug=False):
        """
            battery = [ cell1, cell2, cell3, cell4 ]
        """
        if len(battery) == 4:
            try:
                self.osc_transmitter.send( OSC.OSCMessage('/status/battery', battery))
                if debug: print ' >>> sent battery'
            except OSC.OSCClientError:
                if self.printout: print ' >>> sent battery -> but nobody is listening'
        else:
            print ' >>> battery array too short -> nothing sent'

    def send_keyboard_directions(self, directions=(0, 0, 0, 0), debug=False):
        """
            directions = [ backward-forward, left-right, up-down, left-right-turn ] in range [-1 1]
        """
        if len(directions) == 4:
            try:
                self.osc_transmitter.send(OSC.OSCMessage('/osc_base_station/directions', directions))
                if debug: print ' >>> sent directions'
            except OSC.OSCClientError:
                if self.printout: print ' >>> sent directions -> but nobody is listening'
        else:
            print ' >>> directions array shape wrong short -> nothing sent'

    def send_keyboard_command(self, commands=(), debug=False):
        """
            commands = [ 'cal_acc', 'pLED=1' ]
        """
        if len(commands) > 0:
            try:
                self.osc_transmitter.send(OSC.OSCMessage('/osc_base_station/commands', commands))
                if debug: print ' >>> sent commands', commands
            except OSC.OSCClientError:
                if self.printout: print ' >>> sent commands -> but nobody is listening'
        else:
            print ' >>> commands array shape wrong short -> nothing sent'


def get_local_ip():
    """
        returns a string of the first local ip
    """
    try:
        ip_task = os.popen("ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'")
        local_ip = ip_task.read().strip()
        ip_task.close()
        if '\n' in local_ip:
            local_ip = local_ip.split('\n')[0]
        return local_ip
    except:
        return '0.0.0.0'

# just a short example how this is meant to be used
if __name__ == "__main__":
    osc_receiver = OSC_receiver()
    osc_receiver.start()
    try:
        while True:
            time.sleep(0.1)
            try:
                osc_receiver.osc_rc.calc_rc_from_gyr()
            except:
                print 'error in calc_rc_from_gyr'
    except:
        osc_receiver.stop()
        print 'exit done'
    transmitter = OSC_transmitter("192.168.0.31")
    while True:
        time.sleep(0.2)
        transmitter.send_imu()
        transmitter.send_rc0()
        transmitter.send_rc2()
        transmitter.send_cycletime0()
        transmitter.send_cycletime1()
        transmitter.send_motors()
