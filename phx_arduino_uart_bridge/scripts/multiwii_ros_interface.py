#!/usr/bin/env python
"""
    This script is optimized for performance, therefore no OSC is in use.
    It is meant to publish all flight information coming from multiwii into the ros network.
    At this point of time there is no connection from ros into multiwii possible.
    TODO:
        - make update rate very fast (above 50Hz)
        - publish gps
"""

import pyCopter
print '> import done'

# we first start the copter by connecting to multiwii on a hardcoded serial-port defined in pyCopter.copter_status.init()
phoenix = pyCopter.copter(con_multiwii=True, con_intermediate=False, con_ros=True, osc_transmit=False, osc_receive=False)

# by starting the copter we can now access one 'read-only' remote_control
multiwii_radio = phoenix.serial_multiwii        # this is a 'read-only' RC.

# finally we tell the copter which RC should be used by default
phoenix.use_rc = False

# speed test
phoenix_update_speed_test = pyCopter.speedtest()

try:
    while True:
        # fist thing to do in the main loop is to update all variables of the copter, including updates to the serial clients.
        # This is all done by the pyCopter.update() method. You do not have to worry about timings, just call this line very often.
        # Timings are hardcoded in pyCopter.copter_status.init()
        phoenix_update_speed_test.start()
        phoenix.update()
        phoenix_update_speed_test.stop()
        
        phoenix_update_speed_test.print_result(rate=1., text=' > phoenix update takes')

except KeyboardInterrupt:
    phoenix.stop()
print 'phoenix_os_1.py stopped'
