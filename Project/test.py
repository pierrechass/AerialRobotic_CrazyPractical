import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
            
                keep_flying = True
                move = False
                velocity_y = 0.
                velocity_x = 0.

                while(keep_flying):
                    if is_close(multiranger.up):
                        keep_flying = False
                    
                    print(is_close(multiranger.back))
                    if is_close(multiranger.back):
                        move = ~move
                        if(move):
                            velocity_y = 0.
                            velocity_x = 0.3
                        else : 
                            velocity_y = 0.
                            velocity_x = 0.

                    if is_close(multiranger.front):
                        velocity_x = 0.
                        velocity_y = 0.3
                    else :
                        if(move):
                            velocity_y = 0.
                            velocity_x = 0.3
                        else : 
                            velocity_y = 0.
                            velocity_x = 0.

                    
                    motion_commander.start_linear_motion(velocity_x, velocity_y, 0.)

                    time.sleep(0.1)

                        


            