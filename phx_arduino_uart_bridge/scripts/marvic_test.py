__author__ = 'manuelviermetz'

import pyCopter
import time

mw = pyCopter.serial_com.multiwii_protocol('/dev/marvic')
mw.startup_delay = 10.0

while True:
    time.sleep(0.02)
    mw.receive(debug=False)
    mw.get_msg(cmd_list=[66, 101, 102], debug=False)
#    mw.get_msg(cmd_list=[66], debug=False)
    mw.receive(debug=False)
#    print mw.attitude
    print mw.status
#    print mw.battery
    print mw.raw_imu
    mw.receive(debug=False)
