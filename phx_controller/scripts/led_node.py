#!/usr/bin/env python

import threading
import thread
import struct
import numpy as np

import rospy
from phx_arduino_uart_bridge.msg import Battery
from phx_arduino_uart_bridge.msg import Altitude


class LEDNode:
    file = None
    cur_battery_stat = None
    update_event = None

    def __init__(self):
        self.file = open("/dev/spidev0.1", "w+")
        self.update_event = threading.Event()
        self.animation_update_event = threading.Event()

        # animation parameters
        self.animation_f = 1  # animation frequency
        self.animation_last_state = True

        self.set_led_status(100)

        thread.start_new_thread(self.animation_loop, ())

        rospy.init_node('LEDControl_Node')
        ros_subscribe_battery = rospy.Subscriber('/phx/battery_marvic', Battery, self.callback_battery_update)
        ros_subscribe_altitude_ir = rospy.Subscriber('/phx/sonar_marvic', Altitude, self.callback_altitude_update)

    def callback_battery_update(self, stuff):
        self.cur_battery_stat = stuff
        self.update_event.set()
        self.update_event.clear()

    def callback_altitude_update(self, stuff):
        self.animation_f = abs(1 / ((stuff.estimated_altitude - 10) + 1e-7))
        if self.animation_f > 1:
            self.animation_f = 1

        # self.animation_update_event.set()
        # print (stuff.estimated_altitude)

    def set_led_status(self, status):
        led = np.zeros((4, 3))

        if status == 0:
            led[0] = np.array([65535, 0, 0]) * 1.0
            led[1] = np.array([65535, 0, 0]) * 1.0
            led[2] = np.array([0, 0, 65535]) * 1.0
            led[3] = np.array([0, 0, 65535]) * 1.0
        elif status == 1:
            led[0] = np.array([65535, 65535, 0]) * 1.0
            led[1] = np.array([65535, 65535, 0]) * 1.0
            led[2] = np.array([65535, 65535, 0]) * 1.0
            led[3] = np.array([65535, 65535, 0]) * 1.0

        if status < 100:
            pass # self.set_led_output(led)

    def set_led_animation(self):
        print(1)

    def animation_loop(self):
        led = np.zeros((4, 3))

        while True:
            if not self.animation_last_state:
                led[0] = np.array([65535, 0, 0])
                led[1] = np.array([65535, 0, 0])
                led[2] = np.array([0, 0, 65535])
                led[3] = np.array([0, 0, 65535])
            else:
                led[0] = np.array([0, 0, 0])
                led[1] = np.array([0, 0, 0])
                led[2] = np.array([0, 0, 0])
                led[3] = np.array([0, 0, 0])

            self.set_led_output(led)

            self.animation_last_state = not self.animation_last_state

            self.animation_update_event.wait(self.animation_f)
            self.animation_update_event.clear()

    def set_led_output(self, led):
        # two last bits belong to GS
        self.file.write("\x96\xdf")

        # set rest of GS to max value (=127)
        self.file.write("\xff\xff")

        for (x, y), value in np.ndenumerate(led):
            self.file.write(struct.pack("!H", int(value)))

        self.file.flush()

    def run(self):
        while True:
            self.update_event.wait()

            # TODO read keys and compare values

            if self.cur_battery_stat.cell1 <= 3.5 or self.cur_battery_stat.cell2 <= 3.5 or self.cur_battery_stat.cell3 <= 3.5:
                self.set_led_status(1)
            else:
                self.set_led_status(0)


LEDNode().run()
