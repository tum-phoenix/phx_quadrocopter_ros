#!/usr/bin/env python


import pyCopter
print '> import done'

# we first start the copter by connecting to intermediate_arduino and multiwii.
# At this point also the OSC network connections are established.
# later also the ros_com should be part of pyCopter.
phoenix = pyCopter.copter(con_multiwii=True, con_intermediate=True, con_ros=True, osc_transmit=False, osc_receive=False)

# by starting the copter we can now access three 'read-only' remote_controls
sudo_radio = phoenix.serial_intermediate        # this is a 'read-only' RC.
multiwii_radio = phoenix.serial_multiwii        # this is a 'read-only' RC.
osc_radio = phoenix.osc_remote                  # this is a 'read-only' RC.

# to actually fly the copter computer guided we define a virtual_remote_control which is connected to the copter.
virtual_radio = pyCopter.virtual_remote_control(phoenix)

# finally we tell the copter which RC should be used by default
phoenix.use_rc = virtual_radio

# speed test
phoenix_update_speed_test = pyCopter.speedtest()


try:
    while True:
        # fist thing to do in the main loop is to update all variables of the copter, including updates to the serial clients.
        # This is all done by the pyCopter.update() method. You do not have to worry about timings, just call this line very often.
        phoenix_update_speed_test.start()
        phoenix.update()
        phoenix_update_speed_test.stop()
        
        # piping the OSC radio into the virtual radio
        current_sudo_rc_throttle = sudo_radio.get_stick('throttle')
        # phoenix.osc_remote.calc_rc_from_gyr(throttle=current_sudo_rc_throttle, debug=False)
        if osc_radio:
            phoenix.osc_remote.calc_rc_from_simple_directions(throttle=current_sudo_rc_throttle, debug=False)
            virtual_radio.set_sticks(phoenix.osc_remote.get_sticks(), debug=False)

        virtual_radio.update_rc(debug=False)        # this is not explicitly necessary since phoenix.update() also updates the virtual radio

        phoenix_update_speed_test.print_result(rate=1., text=' > phoenix update takes')

except KeyboardInterrupt:
    phoenix.stop()
print 'phoenix_os_1.py stopped'
