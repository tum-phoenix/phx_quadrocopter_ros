__author__ = 'manuelviermetz'

import pyCopter
import time

mw = pyCopter.serial_com.multiwii_protocol('/dev/multiwii')
mw.startup_delay = 15.0

while True:
    time.sleep(0.25)
    mw.get_msg(cmd_list=[101, 102, 104, 105, 106, 108, 109], debug=False)
    mw.receive(debug=True)
    print mw.attitude
    print mw.status
    print mw.raw_imu