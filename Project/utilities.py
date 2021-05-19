import numpy as np
from cflib.crazyflie.log import LogConfig
import matplotlib.pyplot as plt
from matplotlib import colors

class map_utile():
    def __init__(self, x_start, y_start):
        """
        initialise the map, the map is divided in squares of 1x1 cm
        0 : empty tile in the neutral zone
        1 : obstacle
        2 : start and landing zones
        3 : center of quadcopter
        4 : landing and starting pads
        """
        self.map = np.zeros((50,30))
        self.map[0:14,:] = 2.
        self.map[34:49,:] = 2.
        self.map[x_start,y_start] = 3.

    def update_map_obstacle(self, pos, obstacle = False):
        """
        used to update the map and define the obstacle
        """
        x1,x2 = max(pos[0]-1,0), min(pos[0]+1,49)
        y1,y2 = max(pos[1]-1,0), min(pos[1]+1,29)
        self.map[x1:x2,y1:y2] = 1
        
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
    def update_pad_pos(self, pos, start = True):
        """
        Update pos of the pad on the map
        """
    def plot_map(self):
        # create discrete colormap
        cmap = colors.ListedColormap(['red', 'blue', 'green'])
        bounds = [0,1,2,3]
        norm = colors.BoundaryNorm(bounds, cmap.N)

        fig, ax = plt.subplots()
        ax.imshow(self.map, cmap=cmap, norm=norm)

        # draw gridlines
        ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=0.1)
        ax.set_xticks(np.arange(0.5, 50.5, 1))
        ax.set_yticks(np.arange(0.5, 30.5, 1))

        plt.show()

class calcul():

    def __init__(self):
        pass

    def compute_dist(self):
        """
        Compute distance from sensors measurement
        """
    

class call_backs():
    def __init__(self, cf):
        self._cf = cf

        self.var_z_history = []
        self.var_x_history = []
        self.var_y_history = []

        self.front = []
        self.back = []
        self.right = []
        self.left = []

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        print('Call backs configured')

        return
    
    def _connected(self, link_id):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # The definition of the logconfig can be made before connecting
        
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=20) # We also chhanged the update_period in motion commander
        self._lg_stab.add_variable('stateEstimate.x', 'float')  # estimated X coordinate
        self._lg_stab.add_variable('stateEstimate.y', 'float')  # estimated Y coordinate
        self._lg_stab.add_variable('stateEstimate.z', 'float')  # estimated Z coordinate
        self._lg_stab.add_variable('range.front', 'float')
        self._lg_stab.add_variable('range.back', 'float')
        # self._lg_stab.add_variable('range.left', 'float')
        # self._lg_stab.add_variable('range.right', 'float')

        # self._lg_range = LogConfig(name='ranger', period_in_ms=200) # We also chhanged the update_period in motion commander
        # self._lg_range.add_variable('range.front', 'float')
        # self._lg_range.add_variable('range.back', 'float')
        # self._lg_range.add_variable('range.left', 'float')
        # self._lg_range.add_variable('range.right', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # self._cf.log.add_config(self._lg_range)
            # # This callback will receive the data
            # self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab.data_received_cb.add_callback(self._get_pos)
            # self._lg_range.data_received_cb.add_callback(self._get_range)
            # # This callback will be called on errors
            # self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        return
        
    def _disconnected(self, link_id):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_id)
        return

    def _connection_failed(self, link_id, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_id, msg))
        return

    def _connection_lost(sef, link_id, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_id, msg))

    def _get_pos(self, timestamp, data, logconf):
        """This callback is called when position data are received"""

        self.var_z_history.append(data['stateEstimate.z'])
        self.var_x_history.append(data['stateEstimate.x'])
        self.var_y_history.append(data['stateEstimate.y'])
        self.front.append(data['range.front'])
        self.back.append(data['range.back'])
        # self.right.append(data['range.right'])
        # self.left.append(data['range.left'])

    # def _get_range(self, timestamp, data, logconf):
    #     """This callback is called when range datas are received"""

    #     self.front.append(data['range.front'])
    #     self.back.append(data['range.back'])
    #     self.right.append(data['range.right'])
    #     self.left.append(data['range.left'])
    
    def plot_log(self):
        plt.plot(self.var_z_history)
        plt.show()
