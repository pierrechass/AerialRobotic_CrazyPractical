import logging
import sys
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

from drone import quad

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

cf = Crazyflie(rw_cache='./cache')
drone = quad(cf, pos_init=np.array([100,110,0]))

states = {
    0 :"idle", 
    1 : "take_off",
    2 : "global",
    3 : "local",
    4 : "pad_search",
    5 : "pad_found",
    6 : "land",
    7 : "finished"
    }

def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(uri, cf=cf) as scf:
        drone.connect(scf, verbose=True)
        
        keep_flying = True
        move = False
        velocity_y = 0.
        velocity_x = 0.

        while keep_flying :

            if states.get(drone.state) == 'idle' : 
                drone.take_off()
                time.sleep(0.1)

            if states.get(drone.state) == 'global' :
                drone.global_nav()
                time.sleep(0.1)
            
            if states.get(drone.state) == 'local' :
                drone.local_nav()
                time.sleep(0.1)

            if states.get(drone.state) == 'pad_search' :
                drone.pad_search()
                time.sleep(0.1)
            
            if states.get(drone.state) == 'pad_found' :
                drone.pad_found()
                time.sleep(0.1)
                

            if states.get(drone.state) == 'land':
                drone.land(velocity=0.1)
                time.sleep(0.1)

            if states.get(drone.state) == 'finished':
                print("ploting logs")
                drone.map.plot_map()
                keep_flying = False

        print("finished")