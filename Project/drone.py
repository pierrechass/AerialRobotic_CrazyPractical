import numpy as np
import math as m
import time

# Import cflib and all the necessary
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig

from utilities import *


class quad():

    def __init__(self, cf, id='radio://0/80/2M/E7E7E7E7E7',
                 pos_init=np.array([0, 0, 0]), vel=np.array([0, 0]),
                 state=0, default_heigth=0.3, default_vel=0.2):
        self.pos = pos_init
        self.vel = vel
        self.state = state
        self.default_heigth = default_heigth
        self.default_vel = default_vel
        self.id = id
        self.is_flying = False
        self.cf = cf
        self.cb = call_backs(cf)
        self.map = map_utile(pos_init[0], pos_init[1])
        self.multiranger = None
        self.motion_cmder = None
        self.position_HLcmder = None
        # self.multiranger = Multiranger(scf)
        # self.motion_cmder = MotionCommander(scf,default_height=0.)
        # self.position_HLcmder = PositionHlCommander(scf,
        #                                             x=self.pos[0], y=self.pos[1], z=self.pos[2],
        #                                             default_height=0.)

    def idle(self):
        return self.state

    def take_off(self, heigth=0.3):
        self.motion_cmder.take_off(height=0.3, velocity=self.default_vel)
        # print(taking off)
        self.is_flying = True
        self.state = 4
        return self.state

    def global_nav(self):
        self.motion_cmder
        return self.state

    def local_nav(self):
        return self.state

    def land(self, velocity):
        self.motion_cmder.land(velocity=velocity)
        # print(taking off)
        self.is_flying = False
        self.state = 6
        time.sleep(0.5)
        return self.state

    def pad_search(self):
        #self.motion_cmder.start_linear_motion(0.2,0,0)
        # if self.cb.var_z_history[-2] - self.cb.var_z_history[-1] > 0.2 :
        #     self.map.u
        #time.sleep(5)
        self.motion_cmder.start_linear_motion(0.15,0,0)
        print('straight')
        var=True
        condition2=False
        
        while var==True:
            #print('Z', self.var_z_history[-1])
            if (self.cb.var_z_history[-3]-self.cb.var_z_history[-1]>0.02) & (condition2==False):
                self.pad_x1=self.cb.var_x_history[-1]#pc.land(velocity=0.1)
                print('x1',self.cb.var_x_history[-1])
                print('z1_diff', self.cb.var_z_history[-3]-self.cb.var_z_history[-1])
                condition2=True
                self.motion_cmder.start_linear_motion(0,0,0)
                #print(self.var_z_history[-1])
                time.sleep(2)
                #  print(self.var_z_history[-1])
                self.motion_cmder.start_linear_motion(0.15,0,0)
                #var=False

            #print(self.var_z_history[-1])
            if (self.cb.var_z_history[-1]-self.cb.var_z_history[-3]>0.02): 
                print('cond1')
                if condition2 :
                    print('cond2')
                    if abs(self.cb.var_z_history[-5]-0.3)<0.02:  
                        print('condtkt') 
                        print('x2',self.cb.var_x_history[-1])
                        print('z2_diff', self.cb.var_z_history[-3]-self.cb.var_z_history[-1])
                        self.pad_x2=self.cb.var_x_history[-1]
                        var=False
                    # pc.land(velocity=0.1)
        self.motion_cmder.back(abs(self.pad_x2-self.pad_x1)*2/3, velocity=0.1)
        time.sleep(3)
        self.state = 5
        return self.state

    def emergency_stop(self):
        return

    def update_pos(self):
        self.pos = np.array([self.cb.var_x_history[-1],
                            self.cb.var_y_history[-1],
                            self.cb.var_z_history[-1]])

    def connect(self, scf, verbose=False):
        self.multiranger = Multiranger(scf)
        if verbose:
            print('multiranger connected and configured')
        self.motion_cmder = MotionCommander(scf,
                                            default_height=self.default_heigth)
        if verbose:
            print('Motion Commander connected and configured')
        self.position_HLcmder = PositionHlCommander(scf,
                                                    x=self.pos[0], y=self.pos[1], z=self.pos[2],
                                                    default_velocity=self.default_vel,
                                                    default_height=self.default_heigth)
        if verbose:
            print('Position HL commander connected and configured')

        return
