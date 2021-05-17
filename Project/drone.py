import numpy as np
import math as m

# Import cflib and all the necessary 
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.positioning.position_hl_commander import PositionHlCommander

class quad():

    def __init__(self, cf, pos_init, vel= np.array([0,0]), state = 0):
        self.pos = pos_init
        self.vel = vel
        self.state = state
        self.is_flying = False
        self.multiranger = Multiranger(cf)
        self.motion_cmder = MotionCommander(cf,default_height=0.)
        self.position_HLcmder = PositionHlCommander(cf,
                                                    x=self.pos[0], y=self.pos[1], z=self.pos[2],
                                                    default_height=0.)

    def idle(self):
        return self.state

    def take_off(self):
        self.motion_cmder.take_off(0.1)
        #print(taking off)
        self.is_flying = True
        self.state = 1
        return self.state

    def global_nav(self):
        return self.state
    
    def local_nav(self):
        return self.state
    
    def landing(self):
        return self.state
    
    def pad_search(self):
        return self.state
    
    def emergency_stop(self):
        return

    