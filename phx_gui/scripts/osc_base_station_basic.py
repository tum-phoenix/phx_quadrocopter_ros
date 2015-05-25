__author__ = 'manuelviermetz'

from OSC_com import *

# init osc ----------------------------------------------------------------------------------------------------------------------------------------------------
osc_transmitter = OSCt(destination='192.168.0.35', port=10000)
osc_transmitter.send_connect()
osc_receiver = OSCr(osc_transmitter, port=10001)
osc_receiver.start()

# build osc bridge --------------------------------------------------------------------------------------------------------------------------------------------
# ros       >> osc  >> gui
# subscribe >> send >> receive
osc_receiver.add_receive_message('/phx/altitude_marvic')
osc_receiver.add_receive_message('/phx/status_marvic')
osc_receiver.add_receive_message('/phx/battery_marvic')

# ros     << osc     << gui
# publish << receive << send
publishers = {'/phx/gui_rc': 'Joy',
              '/phx/gui_motors': 'Motor',
              '/phx/gui_parameters': 'GUI_cmd'
              }

counter = 0
try:
    while True:
        time.sleep(1)

        # this is the style to send a topic via osc
        # osc_transmitter.send_topic(topic='/test/topic_to_copter', payload=[100, 100, 200, 200])
        osc_transmitter.send_topic(topic='/phx/gui_parameters', payload=[1, 2, 3, 4, 5, 6, 7, 8])

        print 'osc connection status: OSCr', osc_receiver.connection_status, 'OSCt', osc_transmitter.connection_status, osc_transmitter.sending_failed_counter, time.time() - osc_receiver.time_of_last_keep_alive_receiving
        osc_receiver.keep_alive()
        osc_transmitter.send_connect()
        counter += 1
        print 'sent... ', counter
except:
    osc_receiver.stop()
    osc_transmitter.stop()