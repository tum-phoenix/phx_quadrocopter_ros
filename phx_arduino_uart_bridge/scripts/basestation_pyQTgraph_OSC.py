__author__ = 'manuelviermetz'

import pyCopter
import sys

if len(sys.argv) > 1:
    copter_ip = sys.argv[1]
else:
    copter_ip = '10.152.196.33' # within eduroam
    copter_ip = '192.168.2.1'   # for AccessPoint use

monitor = pyCopter.status_monitor()
osc_transmitter = pyCopter.network_com.OSC_transmitter(destination=copter_ip, port=10001)
osc_transmitter.send_connect()
osc_receiver = pyCopter.network_com.OSC_receiver(osc_base_station=monitor, port=10000)
osc_receiver.start()
monitor.run()
osc_receiver.stop()

print 'exit done'
