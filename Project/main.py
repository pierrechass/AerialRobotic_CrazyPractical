import logging
import sys
import time

# Import cflib and all the necessary 
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# Import all the modules
from drone import *
from utilities import *

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Initialize boolean
demo = True

# Initialize the low-level drivers (don't list the debug drivers)
cflib.crtp.init_drivers(enable_debug_driver=False)
cf = Crazyflie(rw_cache='./cache')

# Initialise FSM's variable
states = {
    0 :"idle", 
    1 : "take_off",
    2 : "global",
    3 : "local",
    4 : "pad_search",
    5 : "land",
    6 : "finished"
    }

state = 0

# Initialise drone
pos_init = np.array([0,0])
drone = quad(cf, pos_init, state = state)

if __name__ == '__main__':

    while(demo):
        """
        Handling of the FSM
        """
        if states(state) == "idle" :
            state = drone.idle()

        elif states(state) == "take_off" :
            state = drone.take_off()
         
        elif states(state) == "global":
            state = drone.global_nav()
        
        elif states(state) == "local":
            state = drone.local_nav()
        
        elif states(state) == "pad_search":
            state = drone.pad_search()

        elif states(state) == "landing" :
            state = drone.landing()

        else :
            drone.emergency_stop()

        


                

                

    