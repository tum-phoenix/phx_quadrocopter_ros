#!/usr/bin/env python
__author__ = 'manuelviermetz'

import pyCopter
print '> import done'


# start an OSC transmitter
osc_out = pyCopter.network_com.OSC_transmitter()

# start a ros node
osc_ros_node = pyCopter.ros_com.ros_communication(osc=osc_out)

# start an OSC receiver
osc_in = pyCopter.network_com.OSC_receiver(osc_rc=osc_ros_node, osc_status_transmitter=osc_out)
osc_in.start()

# speed test
osc_speed_test = pyCopter.speedtest()


try:
    while True:
        osc_speed_test.start()

        osc_ros_node.listen()

        osc_speed_test.stop()

        osc_speed_test.print_result(rate=1., text=' > ros_osc listening takes')
except KeyboardInterrupt:
    osc_out.stop()
    osc_in.stop()
print 'osc_ros_interface.py stopped'
