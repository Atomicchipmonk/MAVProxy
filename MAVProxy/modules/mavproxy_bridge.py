#!/usr/bin/env python
'''module bridge'''

import time, math
import threading
import zmq
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
#from MAVProxy.modules.lib.mp_settings import MPSetting

class BridgeModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(BridgeModule, self).__init__(mpstate, "bridge", "zmqbridge module")
        '''initialisation code'''
        
        self.add_command('sheep', self.cmd_cow, "bah")
        self.add_command('cow', self.cmd_sheep, "moo")
        self.attitude_quaternion = [0,0,0,1]
        self.lat = 0
        self.lon = 0
        self.abs_alt = 0 #absolute in feet
        self.airspeed = 0
        self.rel_alt = 0
        self.heading = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.heading = 0

        lock = threading.Lock()
        # Create new threads
        thread1 = zmqSubThread(self, lock)

        # Start new Threads
        thread1.start()

        print "Exiting Main Thread"




    def handle_attitude(self, m):
        self.roll = m.roll
        self.pitch = m.pitch
        self.yaw = m.yaw
        
        print 'Handle attitude - Roll: %.5f Pitch:.%5f Yaw:%.5f' % (self.roll, self.pitch, self.yaw)
        
    def handle_vfr(self, m):
        self.airspeed = m.airspeed
        self.rel_alt = m.alt
        self.heading = m.heading
        print 'Handle VFR - Speed: %.5f Heading:.%5f RelAlt:%.5f' % (self.airspeed, self.heading, self.rel_alt)
        
    def handle_position(self, m):
        self.lat = m.lat * 1.0e-7
        self.lon = m.lon * 1.0e-7
        self.abs_alt = m.alt * 1.0e-3  #absolute in feet
        print 'Handle position - Lat: %.5f Lon:.%5f AbsAlt:%.3f' % (self.lat, self.lon, self.abs_alt)
        
    def mavlink_packet(self, m):
        '''handle a mavlink packet'''
        type = m.get_type()

        if type in [ 'GPS_RAW', 'GPS_RAW_INT' ]:
            self.handle_position(m)
        elif type == 'ATTITUDE':
            self.handle_attitude(m)
        elif type == 'VFR_HUD':
            self.handle_vfr(m)
            


        
    

        
        
    def cmd_cow(self, args):
        print("cow goes moo")
      
    def cmd_sheep(self, args):
        print("sheep goes bahhhh")

def init(mpstate):
    '''initialise module'''
    return BridgeModule(mpstate)



exitFlag = 0

class zmqSubThread (threading.Thread):
    def __init__(self, safedata, lock):
        threading.Thread.__init__(self)
        self.threadID = 1
        self.name = "zmqsubthread"
        self.safedata = safedata
        self.lock = lock        

        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect ("tcp://localhost:5561")
        socket.setsockopt(zmq.SUBSCRIBE, "TIMER")
        self.socket = socket
    def run(self):
        print "Starting " + self.name

        while True:
            self.socket.recv()
            self.lock.acquire()
            print 'TIMER** - Roll: %.5f Pitch:.%5f Yaw:%.5f' % (self.safedata.roll, self.safedata.pitch, self.safedata.yaw)        
            self.lock.release()

        print "Exiting " + self.name
