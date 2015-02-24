#!/usr/bin/env python


import pyCopter
print '> import done'

# we first start the copter by connecting to intermediate_arduino and multiwii.
# At this point also the OSC network connections are established.
# later also the ros_com should be part of pyCopter.
phoenix = pyCopter.copter(con_multiwii=False, con_intermediate=True, con_ros=True, osc_transmit=False, osc_receive=False)

# by starting the copter we can now access two 'read-only' remote_controls
sudo_radio = phoenix.serial_intermediate    # this is a 'read-only' RC.
ros_radio = phoenix.ros_node                # this is a 'read-only' RC.

# finally we tell the copter which RC should be used by default
phoenix.use_rc = ros_radio

# speed test
phoenix_update_speed_test = pyCopter.speedtest()

try:
    while True:
        # fist thing to do in the main loop is to update all variables of the copter, including updates to the serial clients.
        # This is all done by the pyCopter.update() method. You do not have to worry about timings, just call this line very often.
        phoenix_update_speed_test.start()
        phoenix.update()
        phoenix_update_speed_test.stop()
        
        phoenix_update_speed_test.print_result(rate=1., text=' > phoenix update takes')

except KeyboardInterrupt:
    phoenix.stop()
print 'phoenix_os_1.py stopped'
