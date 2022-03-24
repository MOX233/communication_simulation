from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import csv
from typing import DefaultDict

from utils.read_tripInfo import read_tripInfo
from utils.options import args_parser
from utils.sumo_utils import generate_routefile, sumo_run

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa



class Simulator():
    def __init__(self, W_up, W_down, S, Lambda, T_rd, T_cp, dt, T_total):
        """"
        args: 
            W_up: bandwidth of uplink (in bit/s)
            W_down: bandwidth of downlink (in bit/s)
            S: model size (in bit/model)
            Lambda: arrival rate of cars
            T_rd: round duration
            T_cp: computation delay of cars
            dt: simulation time slot length
            T_total: total simulation time
        """
        self.W_up, self.W_down, self.S, self.Lambda, self.T_rd, self.T_cp = W_up, W_down, S, Lambda, T_rd, T_cp
        self.V_down = W_down/S   # in model/s
        self.V_up = W_up/S       # in model/s
        self.args = args_parser()
        self.args.Lambda = Lambda
        self.args.step_length = dt
        self.args.num_steps = T_total
        self.args.round_duration = T_rd
        

        pass

    def run_sumo(self):
        sumoBinary = checkBinary('sumo')

        # first, generate the route file for this simulation
        generate_routefile(self.args)

        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        traci.start([sumoBinary, "-c", "sumo_data/road.sumocfg",
                                "--tripinfo-output", "sumo_result/tripinfo.xml",
                                '--step-length', str(self.args.step_length),])
        sumo_run(self.args)
        car_tripinfo = read_tripInfo(tripInfo_path='sumo_result/tripinfo.xml')
        import ipdb;ipdb.set_trace()
        pass



sim = Simulator(W_up=1, W_down=1, S=1, Lambda=0.1, T_rd=10, T_cp=5, dt=0.1, T_total=1000)
sim.run_sumo()