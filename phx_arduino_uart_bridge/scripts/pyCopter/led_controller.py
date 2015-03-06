#!/usr/bin/python
__author__ = 'alexanderulanuiski'

import struct
import numpy as np


def update_led(led_setting=None):
    if not led_setting:
        led_setting = np.zeros((4, 3))
    led_controller_file = open("/dev/spidev0.1", "w+")
    led_controller_file.write("\x96\xdf")
    led_controller_file.write("\xff\xff")
    for x in range(0, 4):
        for c in range(0, 3):
            led_controller_file.write(struct.pack("!H", int(led_setting[x, c])))
    led_controller_file.close()
