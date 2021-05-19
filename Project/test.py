import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

from drone import quad

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

cf = Crazyflie(rw_cache='./cache')
drone = quad(cf)

states = {
    0 :"idle", 
    1 : "take_off",
    2 : "global",
    3 : "local",
    4 : "pad_search",
    5 : "land",
    6 : "finished"
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

            if states.get(drone.state) == 'pad_search' :
                drone.pad_search()

            if states.get(drone.state) == 'land':
                drone.land(velocity=0.1)

            if states.get(drone.state) == 'finished':
                print("ploting logs")
                drone.cb.plot_log()
                keep_flying = False
        print("finished")


        # with MotionCommander(scf) as motion_commander:
        #     with Multiranger(scf) as multiranger:
            
        #         keep_flying = True
        #         move = False
        #         velocity_y = 0.
        #         velocity_x = 0.

        #         while(keep_flying):
        #             if is_close(multiranger.up):
        #                 keep_flying = False
                    
        #             print(is_close(multiranger.back))
        #             if is_close(multiranger.back):
        #                 move = ~move
        #                 if(move):
        #                     velocity_y = 0.
        #                     velocity_x = 0.3
        #                 else : 
        #                     velocity_y = 0.
        #                     velocity_x = 0.

        #             if is_close(multiranger.front):
        #                 velocity_x = 0.
        #                 velocity_y = 0.3
        #             else :
        #                 if(move):
        #                     velocity_y = 0.
        #                     velocity_x = 0.3
        #                 else : 
        #                     velocity_y = 0.
        #                     velocity_x = 0.

                    
        #             motion_commander.start_linear_motion(velocity_x, velocity_y, 0.)

        #             time.sleep(0.1)