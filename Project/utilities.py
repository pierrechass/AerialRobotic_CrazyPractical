import numpy as np


class map_utile():
    def __init__(self, x_start, y_start):
        """
        initialise the map, the map is divided in squares of 1x1 cm
        0 : empty tile in the neutral zone
        1 : obstacle
        2 : start and landing zones
        3 : center of quadcopter
        """
        self.map = np.zeros((300,500))
        self.map[:,0:149] = 2.
        self.map[:,449:499] = 2.
        self.map[x_start,y_start] = 3.

    def update_map_obstacle(self, pos, obstacle = False):
        """
        used to update the map and define the obstacle
        """
        
    def update_map_pos(self, pos):
        """
        update the pos of the drone on the map
        """
    def get_neighborhood(self, pos, n_size = 1):
        """
        get the neighborhood of the drone
        """
    
    def update_trajectory(self,pos_init, pos_final, search_pad = False):
        """
        create a new trajectory between pos_init and pos_final
        """

class calcul():

    def __init__(self):
        pass

    def compute_dist(self):
        """
        Compute distance from sensors measurement
        """
    


