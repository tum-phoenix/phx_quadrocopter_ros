__author__ = 'manuelviermetz'

import pyCopter
import time

mw = pyCopter.serial_com.multiwii_protocol('/dev/marvic')
mw.startup_delay = 10.0

while True:
    time.sleep(0.02)
    #mw.get_msg(cmd_list=[101, 102, 104, 105, 106, 108, 109], debug=True)
    mw.get_msg(cmd_list=[66], debug=True)
    mw.receive(debug=True)
    #print mw.attitude
    print mw.status
    print mw.battery
    #print mw.raw_imu
