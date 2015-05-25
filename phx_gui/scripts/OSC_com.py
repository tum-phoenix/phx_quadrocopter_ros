__author__ = 'manuelviermetz'

import os
import time
import OSC
import threading
import sys
import socket
import numpy as np


class OSCr:
    def __init__(self, osc_transmitter=None, port=10000):
        # connecting modules to this osc receiver
        if osc_transmitter:
            self.osc_transmitter = osc_transmitter

        # get own ip address
        self.local_ip = get_local_ip()
        print 'OSC-receiver on', self.local_ip, ':', port

        # set ip addresses
        self.receive_address = self.local_ip, port

        # OSC - CONNECT
        self.osc_receiver = OSC.OSCServer(self.receive_address)         # making this machine a osc receiving server
        self.thread = None                                              # will be started soon

        # connect handlers
        self.osc_receiver.addDefaultHandlers()

        if self.osc_transmitter:
            self.osc_receiver.addMsgHandler("/osc_gui/connect", self.osc_transmitter.callback_connect)
        self.osc_receiver.addMsgHandler("/osc_gui/keep_alive", self.handle_keep_alive)

        # received data is stored to a dictionary
        # { 'message/label': [[time, of, receive, ...], [data, of, receive]], ....}
        self.received_data = {}
        self.receiving_start_time = time.time()
        self.connected_topics = []
        self.time_of_last_keep_alive_receiving = 0
        self.time_of_last_keep_alive_sending = time.time()
        self.connection_status = 'disconnected'

    def restart(self):
        self.stop()
        self.osc_receiver = OSC.OSCServer(self.receive_address)         # making this machine a osc receiving server
        self.thread = None                                              # will be started soon
        self.osc_receiver.addDefaultHandlers()
        if self.osc_transmitter:
            self.osc_receiver.addMsgHandler("/osc_gui/connect", self.osc_transmitter.callback_connect)
        self.osc_receiver.addMsgHandler("/osc_gui/keep_alive", self.handle_keep_alive)
        old_connected_topics = self.connected_topics[:]
        self.connected_topics = []
        for connected_topic in old_connected_topics:
            self.add_receive_message(connected_topic[0], connected_topic[1])
        self.time_of_last_keep_alive_receiving = 0
        self.time_of_last_keep_alive_sending = time.time()
        self.connection_status = 'disconnected'
        self.start()

    def keep_alive(self):
        if self.time_of_last_keep_alive_receiving != 0 and time.time() > self.time_of_last_keep_alive_receiving + 5:
            # no keep alive package received since 5 seconds
            self.restart()
        if self.osc_transmitter and time.time() > self.time_of_last_keep_alive_sending + 1:
            self.time_of_last_keep_alive_sending = time.time()
            self.osc_transmitter.send_keep_alive(debug=False)
    
    def add_receive_message(self, topic, forward_to_ros=False):
        if forward_to_ros:
            self.osc_receiver.addMsgHandler(topic, forward_to_ros)
            self.connected_topics.append([topic, forward_to_ros])
        else:
            self.osc_receiver.addMsgHandler(topic, self.handle_new_message)
            self.connected_topics.append([topic, False])
            self.received_data[topic] = [[], []]
        print ' >>> OSCr added', topic

    def handle_new_message(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/gyro fff [0.48758959770202637, 0.06476165354251862, -0.19856473803520203] ('192.168.0.33', 57527)
        topic = add
        payload = stuff

        self.received_data[topic][0].append(time.time()-self.receiving_start_time)
        if len(self.received_data[topic][1]) == 0:
            for i in range(0, len(payload)):
                self.received_data[topic][1].append([])
        counter = 0
        for load in payload:
            self.received_data[topic][1][counter].append(load)
            counter += 1

    def handle_keep_alive(self, add, tag, stuff, source):
        # input looks like incoming OSC: /gyrosc/gyro fff [0.48758959770202637, 0.06476165354251862, -0.19856473803520203] ('192.168.0.33', 57527)
        self.time_of_last_keep_alive_receiving = time.time()
        self.connection_status = 'connected'

    def start(self):
        print " >>> Starting OSCr on", self.receive_address[0], ':', self.receive_address[1]
        self.thread = threading.Thread(target=self.osc_receiver.serve_forever)
        self.thread.start()
        print ' >>> OSCr started\n'
    
    def stop(self):
        self.osc_receiver.close()
        self.thread.join()
        print ' >>> OSCr closed\n'


class OSCt:
    def __init__(self, destination=None, port=10000, printout=False, ip_change=True):
        """
        give a destination IP and Port to connect to an existing receiver.
        Otherwise the transmitter is waiting for the OSCr to receive a connect package
        :param destination: string
            The IP address to connect to
        :param port:
            The Port to connect to
        :param printout:
        :param ip_change: boolean
            Set to True to allow a received connect package
        :return:
        """
        if destination and port:
            self.destination_ip = destination
            self.destination_port = port
        else:
            self.destination_ip = get_local_ip()
            self.destination_port = port

        self.sending_failed_counter = 0
        self.ip_change = ip_change
        self.printout = printout
        self.destination_address = self.destination_ip, self.destination_port
        print 'OSC-transmitter to', self.destination_ip, ':', self.destination_port

        self.osc_transmitter = OSC.OSCClient()
        self.osc_transmitter.connect(self.destination_address)
        self.time_of_last_connect = 0
        self.connection_status = 'disconnected'

    def stop(self):
        self.osc_transmitter.close()
        print ' >>> OSCt closed'

    def callback_connect(self, add, tag, stuff, source):
        source_ip = source[0]
        if source_ip != self.destination_ip and self.ip_change:
            self.osc_transmitter.close()
            self.destination_ip = source_ip
            self.destination_address = self.destination_ip, self.destination_port
            self.osc_transmitter.connect(self.destination_address)
            print ' >>> new osc transmitter connect'
            print 'OSC-transmitter to', self.destination_ip, ':', self.destination_port

    def send_connect(self, ip=None, debug=False):
        if not ip:
            ip = self.destination_ip
        if debug:
            print ' >>> OSCt sends connect package to', self.destination_ip, ':', self.destination_port
        if time.time() > self.time_of_last_connect + 5:
            self.time_of_last_connect = time.time()
            self.send_topic(topic='/osc_gui/connect', payload=ip, debug=debug)
            print 'sent connect'

    def send_keep_alive(self, debug=False):
        if debug:
            print ' >>> OSCt sends keep_alive package to', self.destination_ip, ':', self.destination_port
        self.send_topic(topic='/osc_gui/keep_alive', payload=[time.time()], debug=debug)

    def send_topic(self, topic, payload, debug=False):
        """
        :param topic: string
            e.g. "/phx/imu"
        :param payload: list or string
            a list of values to be send
        :param debug:
        :return:
        """
        try:
            self.osc_transmitter.send(OSC.OSCMessage(topic, payload))
            self.connection_status = 'connected'
            if debug:
                print ' >>> sent topic', topic, 'with payload', payload
        except OSC.OSCClientError:
            self.sending_failed_counter += 1
            self.connection_status = 'disconnected'
            if self.printout:
                print ' >>> sent topic', topic, '-> but nobody is listening'


def get_local_ip():
    """
        returns a string of the first local ip
    """
    try:
        ip_task = os.popen("ifconfig | grep -Eo 'inet (addr:)?(Adresse:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'")
        local_ip = ip_task.read().strip()
        ip_task.close()
        if '\n' in local_ip:
            local_ip = local_ip.split('\n')[0]
        print ' >> got local ip:', local_ip
        return local_ip
    except:
        return '0.0.0.0'
