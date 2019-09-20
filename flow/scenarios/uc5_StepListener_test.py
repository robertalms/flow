from __future__ import absolute_import
from __future__ import print_function
import sys

import traci
import os
os.environ['SUMO_HOME']="/home/robert/sumo"
from flow.envs import Env
from gym.spaces.box import Box
from gym.spaces.tuple_space import Tuple
import numpy as np
import os
import time
import random
import optparse
import xml.etree.ElementTree as ET
from flow.scenarios import Scenario
from flow.scenarios import UC5_Listener
# the Experiment class is used for running simulations
from flow.core.experiment import Experiment

# the base scenario class
from flow.envs import uc5_env

# all other imports are standard
from flow.core.params import InitialConfig
from flow.core.params import TrafficLightParams
from flow.core.params import VehicleParams
from flow.core.params import NetParams
from flow.core.params import EnvParams
from flow.core.params import SumoParams
##################### Parameters for Scenario and Environment ######################################
# create some default parameters parameters
ADDITIONAL_ENV_PARAMS = {
    "max_accel": 1,
    "max_decel": 1,
}

env_params = EnvParams(additional_params=ADDITIONAL_ENV_PARAMS) 
initial_config = InitialConfig()
vehicles = VehicleParams()

UC5_dir = "/home/robert/flow/examples/UC5/"

sim_params = SumoParams(render=True, sim_step=0.1)
net_params = NetParams(
      template={
        # network geometry features
        "net": os.path.join(UC5_dir, "UC5_1.net.xml"),
        "vtype":[os.path.join(UC5_dir, "vTypesLV_OS.add.xml"),
                 os.path.join(UC5_dir, "vTypesCVToC_OS.add.xml"),
                 os.path.join(UC5_dir, "vTypesCAVToC_OS.add.xml")
                ],
        "flow" : os.path.join(UC5_dir, "routes_trafficMix_0_trafficDemand_1_driverBehaviour_OS_seed_0.xml")
    }
)
ADDITIONAL_SCENARIO_PARAMS = dict(
    downwardEdgeID = "e0",
    distance = 2300.,
    )
    
initial_config = InitialConfig(
    edges_distribution=["e0"],
    additional_params=ADDITIONAL_SCENARIO_PARAMS
)
###################################################################################################

if __name__ == '__main__':
    
    myListener = UC5_Listener(
                            name="template",
                            net_params=net_params,
                            initial_config=initial_config,
                            vehicles=vehicles
                            )

    # create the environment
    env = uc5_env(
        env_params=env_params,
        sim_params=sim_params,
        scenario=myListener,
        simulator='traci',
        stepListener=True
    )
    
    # run the simulation for x steps e.g. 36000 --> step == 0.1sec
    exp = Experiment(env=env)
    _ = exp.run(1, 36000)