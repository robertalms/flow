from __future__ import absolute_import
from __future__ import print_function
import sys
# try:
#     sys.path.append('/home/robert/sumo/tools/')
#     from sumolib import checkBinary  # noqa
# except ImportError:
#     sys.exit(
#         "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")
import traci
import os
from traci.connection import StepListener
os.environ['SUMO_HOME']="/home/robert/sumo"
from flow.envs import Env
from gym.spaces.box import Box
from gym.spaces.tuple_space import Tuple
# experiment imports
import json
import ray
try:
    from ray.rllib.agents.agent import get_agent_class
except ImportError:
    from ray.rllib.agents.registry import get_agent_class
from ray.tune import run_experiments
from ray.tune.registry import register_env
# general imports
import numpy as np
import os
import time
import random
import optparse
import xml.etree.ElementTree as ET
from flow.scenarios import Scenario
from flow.scenarios import UC5_Listener
from flow.core.params import InitialConfig
from flow.core.params import TrafficLightParams

# the Experiment class is used for running simulations
from flow.utils.registry import make_create_env
from flow.utils.rllib import FlowParamsEncoder

# the base scenario class
from flow.scenarios import Scenario
from flow.envs import uc5_env

# all other imports are standard
from flow.core.params import VehicleParams
from flow.core.params import NetParams
from flow.core.params import InitialConfig
from flow.core.params import EnvParams
from flow.core.params import SumoParams
##################### Parameters for Scenario and Environment ######################################
controlled_segments = [("1", 1, True), ("2", 2, True)]
num_observed_segments = [("1", 1), ("2", 3)]

ADDITIONAL_ENV_PARAMS = {
    "controlled_segments": controlled_segments,
    "symmetric": False,
    "observed_segments": num_observed_segments,
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
#     tocInfos = ET.ElementTree(ET.Element("tocInfos")),
    downwardEdgeID = "e0",
    distance = 2300.,
    )    
    
initial_config = InitialConfig(
    edges_distribution=["e0"],
    additional_params=ADDITIONAL_SCENARIO_PARAMS
)
##################### Parameters for BaseRunner-Class #############################################
# Vehicle ID start-string to identify vehicles that are informed by TransAID (none for baseline)
AV_identifiers = [] 
# Map from vehicle ID start-string to identify vehicles, that will perform a ToC in the scenario, 
#     to ToCLead time (in secs) for the corresponding vehicles 
ToC_lead_times = {"CAVToC.":10.0, "CVToC.":0.0}
# Probability, that a given CV or CAV has to perform a ToC
ToCprobability = 1.0
debug=False
##################### Parameters for Experiment #############################################
# time horizon of a single rollout
HORIZON = 1000
# number of parallel workers
N_CPUS = 2
# number of rollouts per training iteration
N_ROLLOUTS = N_CPUS * 4

SCALING = 1
NUM_LANES = 2 * SCALING  # number of lanes in the widest highway
DISABLE_TB = True
DISABLE_RAMP_METER = True
AV_FRAC = 0.10

###################################################################################################
flow_params = dict(
    # name of the experiment
    exp_tag="uc5_tests",

    # name of the flow environment the experiment is running on
    env_name="uc5_env",

    # name of the scenario class the experiment is running on
    scenario="UC5_Listener",

    # simulator that is used by the experiment
    simulator='traci',

    # sumo-related parameters (see flow.core.params.SumoParams)
    sim=SumoParams(
        sim_step=0.1,
        render=True,
        print_warnings=False,
        restart_instance=True,
    ),

    # environment related parameters (see flow.core.params.EnvParams)
    env=EnvParams(
        warmup_steps=40,
        sims_per_step=1,
        horizon=HORIZON,
        additional_params=ADDITIONAL_ENV_PARAMS,
    ),

    # network-related parameters (see flow.core.params.NetParams and the
    # scenario's documentation or ADDITIONAL_NET_PARAMS component)
    net=net_params,
    
    # vehicles to be placed in the network at the start of a rollout (see
    # flow.core.params.VehicleParams)
    veh=vehicles,

    # parameters specifying the positioning of vehicles upon initialization/
    # reset (see flow.core.params.InitialConfig)
    initial=initial_config,
    
    stepListener=True
    

#     # traffic lights to be introduced to specific nodes (see
#     # flow.core.params.TrafficLightParams)
#     tls=traffic_lights,
)


def setup_exps():
    """Return the relevant components of an RLlib experiment.

    Returns
    -------
    str
        name of the training algorithm
    str
        name of the gym environment to be trained
    dict
        training configuration parameters
    """
    alg_run = "PPO"

    agent_cls = get_agent_class(alg_run)
    config = agent_cls._default_config.copy()
    config["num_workers"] = N_CPUS
    config["train_batch_size"] = HORIZON * N_ROLLOUTS
    config["gamma"] = 0.999  # discount rate
    config["model"].update({"fcnet_hiddens": [64, 64]})
    config["use_gae"] = True
    config["lambda"] = 0.97
    config["kl_target"] = 0.02
    config["num_sgd_iter"] = 10
    config['clip_actions'] = False  # FIXME(ev) temporary ray bug
    config["horizon"] = HORIZON

    # save the flow params for replay
    flow_json = json.dumps(
        flow_params, cls=FlowParamsEncoder, sort_keys=True, indent=4)
    config['env_config']['flow_params'] = flow_json
    config['env_config']['run'] = alg_run

    create_env, gym_name = make_create_env(params=flow_params, version=0)

    # Register as rllib env
    register_env(gym_name, create_env)
    return alg_run, gym_name, config


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
    
    
    alg_run, gym_name, config = setup_exps()
    ray.init(num_cpus=N_CPUS + 1, redirect_output=False)
    trials = run_experiments({
        flow_params["exp_tag"]: {
            "run": alg_run,
            "env": gym_name,
            "config": {
                **config
            },
            "checkpoint_freq": 20,
            "checkpoint_at_end": True,
            "max_failures": 999,
            "stop": {
                "training_iteration": 200,
            },
        }
    })
