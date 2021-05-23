from token import RIGHTSHIFT
import numpy as np
import math as m
from sympy.solvers import solve
from sympy import Symbol
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

MIN_DISTANCE = 200
VEL_LOCAL = 0.1
LAND_ZONE = 150

class quad():

    def __init__(self, cf, id='radio://0/80/2M/E7E7E7E7E7',
                 pos_init=np.array([0, 0, 0]), vel=np.array([0, 0]),
                 state=0, default_heigth=0.3, default_vel=0.1):
        self.pos_init = pos_init
        self.pos = pos_init
        self.vel = vel
        self.once = True
        self.attitude = np.array([0,0,0])
        self.yaw = 0
        self.sensors = None
        self.state = state
        self.right = True
        self.default_heigth = default_heigth
        self.default_vel = default_vel
        self.id = id
        self.is_flying = False
        self.cf = cf
        self.cb = call_backs(cf)
        self.map = map_utile(pos_init[0], pos_init[1])
        self.pad = find_pad()
        self.multiranger = None
        self.motion_cmder = None
        self.map.update_pad_pos(self.pos_init[0:2])
        # for searching the pad : 
        self.search_x, self.search_y = self.compute_points_to_search()
        self.pos_ref = np.array([None,None,None])
        self.idx=0
        print(self.search_x,self.search_y)

        # variables to know how to approach the pad when found :
        self.dir=0 # x ou y [0,1]
        self.vel_pad=0 # positif ou negatif 

    def idle(self):
        return self.state

    def take_off(self, heigth=0.3):
        self.motion_cmder.take_off(height=0.3, velocity=self.default_vel)
        self.is_flying = True
        self.state = 2
        time.sleep(2)
        return self.state

    def global_nav(self):
        velocity = 0.2
        #Update pos & sensors
        self.update_pos()

        self.sensors = np.array([self.cb.front[-1], self.cb.right[-1], 
                                 self.cb.back[-1], self.cb.left[-1]])

        self.check_sides()
        self.check_local()

        # if np.isclose(self.pos[0]%20, 0, rtol=1, atol=2.5):
        #     self.check_neigborhood()

        

        if self.sensors[-2] < MIN_DISTANCE :
            self.state = 6
        
        if 500 - self.pos[0] < LAND_ZONE :
            self.pad_search()
            velocity = 0.3
        else :
            self.pos_ref = [None,0,None]

        self.update_velocity(pos_ref_x=self.pos_ref[0],pos_ref_y=self.pos_ref[1],vel_x=self.vel[0], vel_y=self.vel[1], velocity = velocity)
        return self.state

    def local_nav(self):
        self.update_pos()
        self.check_sides()
        self.sensors = np.array([self.cb.front[-1], 
                                 self.cb.back[-1]])

        if(self.cb.front[-1] <= MIN_DISTANCE):
            self.vel = np.array([0,self.vel_local,0])
            pos_obstacle = [int(self.pos[0] + self.sensors[0]/10),int(self.pos[1])]
            self.map.update_map_obstacle(pos_obstacle)
        else :
            self.vel = np.array([0,0,0])
            self.state = 2
        
        self.motion_cmder.start_linear_motion(self.vel[0], self.vel[1], self.vel[2])
        return self.state

    def land(self, velocity):
        self.motion_cmder.land(velocity=velocity)
        self.is_flying = False
        self.state = 7
        self.map.update_pad_pos(self.pos[0:2])
        time.sleep(0.5)
        return self.state

    def pad_search(self, verbose = False):
        self.zigzag()
        # If pad found then go to state path_found (5)
        if self.pad.find_pad_out(self.cb.var_z_history,self.cb.var_x_history,self.cb.var_y_history):      
            self.state=5
            self.motion_cmder.start_linear_motion(0,0,0) 
            time.sleep(3)

    def pad_found(self):
        state = 1
        print('state',state)
        # Algo for landing in the pad
        while state !=4:
            self.update_pos()
            if self.pad.states_pad.get(state)=="border1":
                print('border1')
                if self.dir==0:
                    self.update_velocity(pos_ref_x=None,pos_ref_y=self.pad.pad_x1[1],
                                        vel_x=self.vel_pad*0.12,vel_y=None)
                else:                   
                    self.update_velocity(pos_ref_x=self.pad.pad_x1[0],pos_ref_y=None,
                                        vel_x=None,vel_y=self.vel_pad*0.12)
                if self.pad.find_pad_in(self.cb.var_z_history,self.cb.var_x_history,self.cb.var_y_history,var=0):
                    time.sleep(0.5)
                    self.motion_cmder.start_linear_motion(0,0,0) 
                    time.sleep(1)
                    state=2
                    if self.dir==0:
                        mid_pad_x=self.pad.pad_x1[0]+1/2*(self.pad.pad_x2[0]-self.pad.pad_x1[0])
                        self.go_to(x_ref=mid_pad_x,y_ref=self.pad.pad_x1[1],z_ref=None)
                    else:
                        mid_pad_y=self.pad.pad_x1[1]+1/2*(self.pad.pad_x2[1]-self.pad.pad_x1[1])
                        self.go_to(x_ref=self.pad.pad_x1[0],y_ref=mid_pad_y,z_ref=None)

            if self.pad.states_pad.get(state)=="border2":
                print('border2')
 
                if self.dir==0:
                    self.update_velocity(pos_ref_x=mid_pad_x,pos_ref_y=None,vel_x=None,vel_y=-0.12)
                else:
                    self.update_velocity(pos_ref_x=None,pos_ref_y=mid_pad_y,vel_x=-0.12,vel_y=None)
                time.sleep(0.2)
                if self.pad.find_pad_in(self.cb.var_z_history,self.cb.var_x_history,self.cb.var_y_history,var=1):
                    time.sleep(0.5)
                    self.motion_cmder.start_linear_motion(0,0,0) 
                    time.sleep(1)
                    if self.dir==0:
                        self.go_to(x_ref=mid_pad_x,y_ref=self.pad.pad_x1[1],z_ref=None)
                    else:
                        self.go_to(x_ref=self.pad.pad_x1[0],y_ref=mid_pad_y,z_ref=None)
                    state=3

            if self.pad.states_pad.get(state)=="border3":
                print('border3')
                if self.dir==0:    
                    self.update_velocity(pos_ref_x=mid_pad_x,pos_ref_y=None,vel_x=None,vel_y=0.12)
                else:
                    self.update_velocity(pos_ref_x=None,pos_ref_y=mid_pad_y,vel_x=0.12,vel_y=None)
                time.sleep(0.2)
                if self.pad.find_pad_in(self.cb.var_z_history,self.cb.var_x_history,self.cb.var_y_history,var=2):  
                    time.sleep(0.5)
                    self.motion_cmder.start_linear_motion(0,0,0) 
                    time.sleep(1)
                    if self.dir==0:
                        mid_pad_y=self.pad.pad_y1[1]+1/2*(self.pad.pad_y2[1]-self.pad.pad_y1[1])
                    else :
                        mid_pad_x=self.pad.pad_x1[0]+1/2*(self.pad.pad_x2[0]-self.pad.pad_x1[0])

                    self.go_to(x_ref=mid_pad_x,y_ref=mid_pad_y,z_ref=None)

                    print('pad mid x',mid_pad_x)
                    print('pad mid y',mid_pad_y)

                    print('pos_x',self.cb.var_x_history[-1])
                    print('pos_y',self.cb.var_y_history[-1])
                    time.sleep(0.5)
                    self.go_to(x_ref=mid_pad_x,y_ref=mid_pad_y,z_ref=0.2)
                    state=4

        self.state = 6
        return

    def update_pos(self):
        self.pos = np.array([self.cb.var_x_history[-1]*100 + self.pos_init[0],
                            self.cb.var_y_history[-1]*100 + self.pos_init[1],
                            self.cb.var_z_history[-1]*100 + self.pos_init[2]])

    def connect(self, scf, verbose=False):
        self.motion_cmder = MotionCommander(scf,
                                            default_height=self.default_heigth)
        if verbose:
            print('Motion Commander connected and configured')
        # self.position_HLcmder = PositionHlCommander(scf,
        #                                             x=self.pos[0], y=self.pos[1], z=self.pos[2],
        #                                             default_velocity=self.default_vel,
        #                                             default_height=self.default_heigth)
        # if verbose:
        #     print('Position HL commander connected and configured')

        self.multiranger = Multiranger(scf)
        if verbose:
            print('multiranger connected and configured')
        return

    def compute_vel_local(self):
        """
        Compute in which direction should the drone go in local avoidance
        """

        distx0 = self.pos[0]
        distx500 = 500 - self.pos[0]
        disty0 = self.pos[1]
        disty300 = 300 - self.pos[1]

        if m.isclose(self.attitude[0]%360,0) :
            if disty0 > disty300 :
                vel = -VEL_LOCAL
            else :
                vel = VEL_LOCAL

        elif m.isclose(self.attitude[0]%360,180):
            if disty0 > disty300 :
                vel = VEL_LOCAL
            else :
                vel = -VEL_LOCAL
        elif m.isclose(self.attitude[0]%360,90):
            if distx0 > distx500 :
                vel = VEL_LOCAL
            else :
                vel = -VEL_LOCAL

        elif m.isclose(self.attitude[0]%360,-90):
            if distx0 > distx500 :
                vel = -VEL_LOCAL
            else :
                vel = VEL_LOCAL

        else : 
            a = m.tan(m.radians(self.attitude[0]))
            x = Symbol('x')
            y = Symbol('y')
            x1 = self.pos[0]
            y1 = self.pos[1]
            b = Symbol('b')

            # Find b
            b = solve(a*x1 + b - y1, b)
            
            # Find all distances to each axes of the map
            x0 = b[0]
            x500 = solve(a*500 + b[0] -y, y)[0]
            y0 = solve(a*x + b[0],x)[0]
            y300 = solve(a*x + b[0] + 300,x)[0]

            idx = np.argmin((np.clip([x0,x500,y0,y300],0,None)))

            if idx == 0 : 
                if self.attitude[0]%360 > 0 & self.attitude[0]%360 < 180 :
                    vel = VEL_LOCAL
                else :
                    vel = -VEL_LOCAL
            if idx == 1 :
                if self.attitude[0]%360 > 0 & self.attitude[0]%360 < 180 :
                    vel = -VEL_LOCAL
                else :
                    vel = VEL_LOCAL
            if idx == 2 :
                if self.attitude[0]%360 > -90 & self.attitude[0]%360 < 90 :
                    vel = -VEL_LOCAL
                else :
                    vel = VEL_LOCAL
            if idx == 3 :
                if self.attitude[0]%360 > -90 & self.attitude[0]%360 < 90 :
                    vel = VEL_LOCAL
                else :
                    vel = -VEL_LOCAL
        
        self.vel_local = vel

    def check_local(self):
        if(self.cb.front[-1] >= MIN_DISTANCE):
            self.vel = np.array([0.3,None,None])
            obstacle = False
        else :
            self.compute_vel_local()
            self.vel = np.array([None,None,None])
            self.state = 3
            obstacle = True
        return obstacle
    
    def check_neigborhood(self):
        i = 1
        obstacle = False
        if self.right :
            while((i != 10) & (self.cb.right[-1] > MIN_DISTANCE)):
                self.motion_cmder.start_linear_motion(0,0.1,0)
                if self.check_local():
                    obstacle = True
                time.sleep(0.1)
                i+=1
            self.motion_cmder.start_linear_motion(0,0,0)
            if  not obstacle :
                self.motion_cmder.start_linear_motion(0,-0.1,0)
                time.sleep(0.4)
            self.right = not self.right
        else :
            while((i != 10) & (self.cb.left[-1] > MIN_DISTANCE)):
                if self.cb.left[-1] > MIN_DISTANCE :
                    self.motion_cmder.start_linear_motion(0,-0.1,0)
                    if self.check_local():
                        obstacle = True
                    time.sleep(0.1)
                    i+=1
            self.motion_cmder.start_linear_motion(0,0,0)
            if not obstacle :
                self.motion_cmder.start_linear_motion(0,0.1,0)
                time.sleep(0.4)
            self.right = not self.right
    
    def check_sides(self):
        if self.cb.right[-1] < MIN_DISTANCE :
            pos_obstacle = [int(self.pos[0]),int(self.pos[1] - self.cb.right[-1]/5)]
            self.map.update_map_obstacle(pos_obstacle)

        if self.cb.left[-1] < MIN_DISTANCE :
            pos_obstacle = [int(self.pos[0]),int(self.pos[1] + self.cb.left[-1]/5)]
            self.map.update_map_obstacle(pos_obstacle)

    def go_to(self,x_ref,y_ref,z_ref=None):

        x = self.cb.var_x_history[-1]
        y = self.cb.var_y_history[-1]
        distance = m.sqrt(((x_ref-x)**2)+((y_ref-y)**2))
        
        while (distance>0.01):
            self.update_velocity(pos_ref_x=x_ref, pos_ref_y=y_ref, pos_ref_z=z_ref,
                                 vel_x=None, vel_y=None, vel_z=None)
            x = self.cb.var_x_history[-1]
            y = self.cb.var_y_history[-1]
            distance = m.sqrt(((x_ref-x)**2)+((y_ref-y)**2))

    def update_velocity(self, pos_ref_x=None, pos_ref_y=None, pos_ref_z=None,
                        vel_x=None, vel_y=None, vel_z=None,velocity=0.2):
        update_vel=[0,0,0]
        coeff=1

        if pos_ref_x is None:
            if vel_x is None: 
                update_vel[0]=0
            else : 
                update_vel[0]=vel_x

        elif vel_x is None:
            if (pos_ref_x-self.cb.var_x_history[-1])>0:
                update_vel[0]=min(velocity, (pos_ref_x-self.cb.var_x_history[-1])*coeff)
            else:
                update_vel[0]=max(-velocity, (pos_ref_x-self.cb.var_x_history[-1])*coeff)

        if pos_ref_y is None:
            if vel_y is None:
                update_vel[1]=0
            else : 
                update_vel[1]=vel_y

        elif vel_y is None:
            if (pos_ref_y-self.cb.var_y_history[-1])>0:
                update_vel[1]=min(velocity, (pos_ref_y-self.cb.var_y_history[-1])*coeff)
            else:
                update_vel[1]=max(-velocity, (pos_ref_y-self.cb.var_y_history[-1])*coeff)

        if pos_ref_z is None:
            if vel_z is None:
                update_vel[2]=0
            else : 
                update_vel[2]=vel_z

        elif vel_z is None:
            if (pos_ref_z-self.cb.var_z_history[-1])>0:
                update_vel[2]=min(0.15, (pos_ref_z-self.cb.var_z_history[-1])*coeff)
            else:
                update_vel[2]=max(-0.15, (pos_ref_z-self.cb.var_z_history[-1])*coeff)

        new_vel_x=update_vel[0]*m.cos(self.yaw)-update_vel[1]*m.sin(self.yaw)
        new_vel_y=update_vel[1]*m.cos(self.yaw)+update_vel[0]*m.sin(self.yaw)
        self.motion_cmder.start_linear_motion(new_vel_x,new_vel_y,update_vel[2]) 

        time.sleep(0.2)

    def zigzag(self):
        # assume that we are at the middle 
        self.pos_ref = np.array([self.search_x[self.idx], self.search_y[self.idx], None])
        self.vel = [None,None,None]
        x = self.cb.var_x_history[-1]
        y = self.cb.var_y_history[-1]
        distance = m.sqrt(((self.search_x[self.idx]-x)**2)+((self.search_y[self.idx]-y)**2))
        print(distance)
        if distance<0.02:
            self.idx=self.idx+1        
    def compute_points_to_search(self):
        search_x = []
        search_y = []
        step = 0.4

        search_x.append(3.5-self.pos_init[0]//100)
        search_y.append(0-self.pos_init[1]//100)

        for i in range(0,8,2):
            search_x.append(3.5+i*step-self.pos_init[0]//100)
            search_y.append(2.4-self.pos_init[1]//100)

            search_x.append(3.5+(i+1)*step-self.pos_init[0]//100)
            search_y.append(2.4-self.pos_init[1]//100)

            search_x.append(3.5+(i+1)*step-self.pos_init[0]//100)
            search_y.append(0-self.pos_init[1]//100)

            search_x.append(3.5+(i+2)*step-self.pos_init[0]//100)
            search_y.append(0-self.pos_init[1]//100)

        return search_x, search_y
        
        