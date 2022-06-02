import datetime
import json
import time

import sys
import os
import configparser
import rclpy

import numpy as np
import graph_ltpl

# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist

# -- Limit number of OPENBLAS library threads --
# On linux based operation systems, we observed a occupation of all cores by the underlying openblas library. Often,
# this slowed down other processes, as well as the planner itself. Therefore, it is recommended to set the number of
# threads to one. Note: this import must happen before the import of any openblas based package (e.g. numpy)
os.environ['OPENBLAS_NUM_THREADS'] = str(1)


"""
This is the main script to run a standard example of the graph-based local trajectory planner.

:Authors:
    * Tim Stahl <tim.stahl@tum.de>

:Created on:
    18.08.2020
"""

# ----------------------------------------------------------------------------------------------------------------------
# IMPORT (should not change) -------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------


class Publisher(Node):
    def __init__(self):
        super().__init__('final_publisher')
        # top level path (module directory)
        toppath = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(toppath)
        print("TOPPPATH: ", toppath)
        track_param = configparser.ConfigParser()
        if not track_param.read(toppath + "/driving_task.ini"):
            raise ValueError('Specified online parameter config file does not exist or is empty!')

        track_specifier = json.loads(track_param.get('DRIVING_TASK', 'track'))

        # define all relevant paths
        path_dict = {'globtraj_input_path': toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + track_specifier + ".csv",
                     'graph_store_path': toppath + "/inputs/stored_graph.pckl",
                     'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
                     'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini",
                     'log_path': toppath + "/logs/graph_ltpl/",
                     'graph_log_id': datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
                     }
        print('Here 1')
        # ----------------------------------------------------------------------------------------------------------------------
        # INITIALIZATION AND OFFLINE PART --------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------

        # intialize graph_ltpl-class
        self.ltpl_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=path_dict,
                                                         visual_mode=True,
                                                         log_to_file=True)

        print('Here 2')
        # calculate offline graph
        self.ltpl_obj.graph_init()

        print('Here 3')
        # set start pose based on first point in provided reference-line
        refline = graph_ltpl.imp_global_traj.src.import_globtraj_csv.\
            import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]
        print('Here 4')
        self.pos_est = refline[0, :]
        print('Here 5')
        heading_est = np.arctan2(np.diff(refline[0:2, 1]), np.diff(refline[0:2, 0])) - np.pi / 2

        print('Here 6')
        self.vel_est = 0.0

        # set start pos
        self.ltpl_obj.set_startpos(pos_est=self.pos_est,
                                   heading_est=heading_est)
        print('Here 7')

        # ----------------------------------------------------------------------------------------------------------------------
        # ONLINE LOOP ----------------------------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------

        # init dummy object list
        self.obj_list_dummy = graph_ltpl.testing_tools.src.objectlist_dummy.ObjectlistDummy(dynamic=True,
                                                                                            vel_scale=0.3,
                                                                                            s0=250.0)
        print('Here 8')

        # init sample zone (NOTE: only valid with the default track and configuration!)
        # INFO: Zones can be used to temporarily block certain regions (e.g. pit lane, accident region, dirty track, ....).
        #       Each zone is specified in a as a dict entry, where the key is the zone ID and the value is a list with the cells
        #        * blocked layer numbers (in the graph) - pairwise with blocked node numbers
        #        * blocked node numbers (in the graph) - pairwise with blocked layer numbers
        #        * numpy array holding coordinates of left bound of region (columns x and y)
        #        * numpy array holding coordinates of right bound of region (columns x and y)
        self.zone_example = {'sample_zone': [[64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66],
                                             [0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6],
                                             np.array([[-20.54, 227.56], [23.80, 186.64]]),
                                             np.array([[-23.80, 224.06], [20.17, 183.60]])]}
        print('Here 9')

        self.traj_set = {'straight': None}
        self.tic = time.time()

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
# callback

# while True:
    # -- SELECT ONE OF THE PROVIDED TRAJECTORIES -----------------------------------------------------------------------
    # (here: brute-force, replace by sophisself.ticated behavior planner)
        for sel_action in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
            if sel_action in self.traj_set.keys():
                return

        # get simple object list (one vehicle driving around the track)
        obj_list = self.obj_list_dummy.get_objectlist()

        # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
        self.ltpl_obj.calc_paths(prev_action_id=sel_action,
                                 object_list=obj_list,
                                 blocked_zones=self.zone_example)

        # -- GET POSITION AND VELOCITY ESTIMATE OF EGO-VEHICLE -------------------------------------------------------------
        # (here: simulation dummy, replace with actual sensor readings)
        if self.traj_set[sel_action] is not None:
            self.pos_est, self.vel_est = graph_ltpl.testing_tools.src.vdc_dummy.\
                vdc_dummy(pos_est=self.pos_est,
                          last_s_course=(self.traj_set[sel_action][0][:, 0]),
                          last_path=(self.traj_set[sel_action][0][:, 1:3]),
                          last_vel_course=(self.traj_set[sel_action][0][:, 5]),
                          iter_time=time.time() - self.tic)
        self.tic = time.time()

        # -- CALCULATE VELOCITY PROFILE AND RETRIEVE TRAJECTORIES ----------------------------------------------------------
        self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.pos_est,
                                                       vel_est=self.vel_est)[0]

        # -- SEND TRAJECTORIES TO CONTROLLER -------------------------------------------------------------------------------
        # select a trajectory from the set and send it to the controller here

        # -- LOGGING -------------------------------------------------------------------------------------------------------
        self.ltpl_obj.log()

        # -- LIVE PLOT (if activated - not recommended for performance use) ------------------------------------------------
        # self.ltpl_obj.visual()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        # Publish the message to the topic
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    final_publisher = Publisher()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(final_publisher)
    # Explicity destroy the node
    final_publisher.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
