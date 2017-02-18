#!/usr/bin/env python
'''module bridge'''

import time, math
import threading
import zmq
import sys, os
import signal
import utm

from math import cos, sin, radians, degrees, atan2, sqrt, acos, pi

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
#from MAVProxy.modules.lib.mp_settings import MPSetting

class BridgeModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(BridgeModule, self).__init__(mpstate, "bridge", "zmqbridge module")
        '''initialisation code'''
        
        self.add_command('sheep', self.cmd_cow, "bah")
        self.add_command('cow', self.cmd_sheep, "moo")
        self.attitude_quaternion = [0,0,0,0]
        self.lat = 0
        self.lon = 0
        self.x = 0
        self.y = 0
        self.abs_alt = 0 #absolute in feet
        self.airspeed = 0
        self.rel_alt = 0
        self.heading = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.heading = 0

        stopper = threading.Event()
        self.stopper = stopper
        lock = threading.Lock()
        self.lock = lock

        # Create new threads
        thread1 = zmqSubThread(self, lock, stopper)
        #thread1.daemon = True
        #Create the handler
        workers = [thread1]
#        handler = SignalHandler(stopper, workers)
#        signal.signal(signal.SIGINT, handler)

        # Start new Threads
        thread1.start()

        print "Exiting Main Thread"




    def handle_attitude(self, m):
        self.lock.acquire()
        self.roll = m.roll
        self.pitch = m.pitch
        self.yaw = m.yaw
        #self.attitude_quaternion = self.toQuaternion(self.roll, self.pitch, self.yaw)
        #quaternion math
#        print "changing attitude"
        t0 = cos(self.yaw * 0.5);
        t1 = sin(self.yaw * 0.5);
        t2 = cos(self.roll * 0.5);
        t3 = sin(self.roll * 0.5);
        t4 = cos(self.pitch * 0.5);
        t5 = sin(self.pitch * 0.5);


        self.attitude_quaternion[0] = t0 * t3 * t4 - t1 * t2 * t5;
        self.attitude_quaternion[1] = t0 * t2 * t5 + t1 * t3 * t4;
        self.attitude_quaternion[2] = t1 * t2 * t4 - t0 * t3 * t5;
        self.attitude_quaternion[3] = t0 * t2 * t4 + t1 * t3 * t5;





        self.lock.release()

    def toQuaternion(self, roll, pitch, yaw):
        '''defined as 
        in -> roll pitch yaw
        out <- x y z w '''


        t0 = cos(yaw * 0.5);
        t1 = sin(yaw * 0.5);
        t2 = cos(roll * 0.5);
        t3 = sin(roll * 0.5);
        t4 = cos(pitch * 0.5);
        t5 = sin(pitch * 0.5);


        self.attitude_quaternion[0] = t0 * t3 * t4 - t1 * t2 * t5;
        self.attitude_quaternion[1] = t0 * t2 * t5 + t1 * t3 * t4;
        self.attitude_quaternion[2] = t1 * t2 * t4 - t0 * t3 * t5;
        self.attitude_quaternion[3] = t0 * t2 * t4 + t1 * t3 * t5;

        return q;


    def handle_vfr(self, m):
        self.lock.acquire()
        self.airspeed = m.airspeed
        self.rel_alt = m.alt
        self.heading = m.heading
        self.lock.release()

    def handle_position(self, m):
        self.lock.acquire()
        self.lat = m.lat * 1.0e-7
        self.lon = m.lon * 1.0e-7
        utm_out = utm.from_latlon(self.lat, self.lon)
        self.y = utm_out[0]
        self.x = utm_out[1]
        self.abs_alt = m.alt * 1.0e-3  #absolute in feet
        self.lock.release()

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
    def __init__(self, safedata, lock, stopper):
        threading.Thread.__init__(self)
        self.threadID = 1
        self.name = "zmqsubthread"
        self.safedata = safedata
        self.lock = lock        
        self.stopper = stopper

        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect ("tcp://localhost:5561")
        socket.setsockopt(zmq.SUBSCRIBE, "TIMER")
        self.socket = socket

        f = open('/home/pi/captures/current/telemetery.csv','w')
        self.file = f
    def run(self):
        print "Starting " + self.name

        self.file.write('Time,Altitude,Position_1,Position_2,Position_3,Velocity_1,Velocity_2,Velocity_3,q_I_to_B_1,q_I_to_B_2,q_I_to_B_3,q_I_to_B_4,Sun_Vector_1,Sun_Vector_2,Sun_Vector_3,q_I_to_T_1,q_I_to_T_2,q_I_to_T_3,q_I_to_T_4,q_I_to_B_cov_11,q_I_to_B_cov_12,q_I_to_B_cov_13,q_I_to_B_cov_21,q_I_to_B_cov_22,q_I_to_B_cov_23,q_I_to_B_cov_31,q_I_to_B_cov_32,q_I_to_B_cov_33\n')
        while not self.stopper.is_set():
            packet = self.socket.recv_multipart()
            #topic, messagedata = packet.split()
            time =  int(packet[1])
            self.lock.acquire()
            
            #time, altitude, position x, y, z, velocity x, y ,z, quat 0, 1, 2, 3, extra
            self.file.write('%s,%f,%f,%f,%f,0,0,0,%f,%f,%f,%f,0,0.707106781,0.707106781,0,0,0,1,0,0,0,0.00000000504301,0,0,0,0.00000000504301\n' % (time, self.safedata.rel_alt, self.safedata.x, self.safedata.y, self.safedata.abs_alt, self.safedata.attitude_quaternion[0],self.safedata.attitude_quaternion[1],self.safedata.attitude_quaternion[2],self.safedata.attitude_quaternion[3]))

            #print 'Attitude - Roll: %.5f Pitch:.%5f Yaw:%.5f' % (self.safedata.roll, self.safedata.pitch, self.safedata.yaw)        
            #print 'Quaternion - x:%.3f y:%.3f z:%.3f w:%.3f' % (self.safedata.attitude_quaternion[0], self.safedata.attitude_quaternion[1], self.safedata.attitude_quaternion[2], self.safedata.attitude_quaternion[3])
            #print 'VFR - Speed: %.5f Heading:.%5f RelAlt:%.5f' % (self.safedata.airspeed, self.safedata.heading, self.safedata.rel_alt)
            #print 'Position - Lat: %.5f Lon:.%5f AbsAlt:%.3f' % (self.safedata.lat, self.safedata.lon, self.safedata.abs_alt)
            self.lock.release()
            self.file.flush()
            os.fsync(self.file.fileno())
        print "Exiting " + self.name
        self.file.close()

class SignalHandler:
    """    The object that will handle signals and stop the worker threads."""

    #: The stop event that's shared by this handler and threads.
    stopper = None

    #: The pool of worker threads
    workers = None

    def __init__(self, stopper, workers):
        self.stopper = stopper
        self.workers = workers

    def __call__(self, signum, frame):
        """        This will be called by the python signal module https://docs.python.org/3/library/signal.html#signal.signal"""
        self.stopper.set()

        for worker in self.workers:
            worker.join()

        sys.exit(0)

