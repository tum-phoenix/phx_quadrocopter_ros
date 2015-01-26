__author__ = 'manuelviermetz'

from SERIAL_com import *
import time

class serial_rc:
    """
        OLD this is going to be deleted soon!
    """
    def __init__( self, serial_protocol, send_rate=10 ):
        self.serial = serial_protocol
        self.send = True
        self.send_interval = 1./send_rate
        self.time_of_next_update = 0
        self.throttle = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.aux1 = 0
        self.aux2 = 0
        self.aux3 = 0
        self.aux4 = 0
    
    def update( self , debug=False):
        if ( ( time.time() >= self.time_of_next_update ) and ( self.send ) ) :
            self.time_of_next_update = time.time() + self.send_interval
            try:
                sticks = [ self.throttle, self.pitch, self.roll, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4 ]
                self.serial.send_rc( sticks, debug=False )
                if debug: print ' >> serial_rc sent',sticks
            except:
                print ' >>> ERROR in serial_rc sending',sticks
    
    def set( self, throttle, pitch, roll, yaw, aux1, aux2, aux3, aux4):
        if throttle!=None: self.throttle = throttle
        if pitch!=None: self.pitch = pitch
        if roll!=None: self.roll = roll
        if yaw!=None: self.yaw = yaw
        if aux1!=None: self.aux1 = aux1
        if aux2!=None: self.aux2 = aux2
        if aux3!=None: self.aux3 = aux3
        if aux4!=None: self.aux4 = aux4
