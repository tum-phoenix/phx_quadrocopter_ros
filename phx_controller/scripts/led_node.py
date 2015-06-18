#!/usr/bin/env python

import threading
import struct
import numpy as np

import rospy
from phx_arduino_uart_bridge.msg import Battery


class LEDNode:

    file = None
    cur_battery_stat = None
    update_event = None

    def __init__(self):
        self.file = open("/dev/spidev0.1", "w+")
        self.update_event = threading.Event()
        self.set_led_status(0)

        rospy.init_node('LEDControl_Node')
        ros_subscribe_battery = rospy.Subscriber('/phoenix/stat_battery', Battery, self.callback_battery_update)

    def callback_battery_update(self, stuff):
        self.cur_battery_stat = stuff
        self.update_event.set()
        self.update_event.clear()

    def set_led_status(self, status):
        led = np.zeros((4, 3))

        if status == 0:
            led[0] = np.array([65535, 0, 0])*1.0
            led[1] = np.array([65535, 0, 0])*1.0
            led[2] = np.array([0, 0, 65535])*1.0
            led[3] = np.array([0, 0, 65535])*1.0
        elif status == 1:
            led[0] = np.array([65535, 65535, 0])*1.0
            led[1] = np.array([65535, 65535, 0])*1.0
            led[2] = np.array([65535, 65535, 0])*1.0
            led[3] = np.array([65535, 65535, 0])*1.0

        self.set_led_output(led)

    def set_led_output(self, led):
        # two last bits belong to GS
        self.file.write("\x96\xdf")

        # set rest of GS to max value (=127)
        self.file.write("\xff\xff")

        for (x,y), value in np.ndenumerate(led):
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
