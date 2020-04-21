"""I-210 subnetwork example.

In this case flow_params is a list of dicts. This is to test the effects of
multiple human-driver model parameters on the flow traffic.
"""
from collections import OrderedDict
from copy import deepcopy
import itertools
import os
import numpy as np

from flow.core.params import SumoParams
from flow.core.params import EnvParams
from flow.core.params import NetParams
from flow.core.params import SumoLaneChangeParams
from flow.core.params import VehicleParams
from flow.core.params import InitialConfig
from flow.core.params import InFlows
import flow.config as config
from flow.envs import TestEnv
from flow.networks.i210_subnetwork import I210SubNetwork, EDGES_DISTRIBUTION

# the default parameters for all lane change parameters
default_dict = {
    "lane_change_mode": "strategic",
    "model": "LC2013",
    "lc_strategic": 1.0,
    "lc_cooperative": 1.0,
    "lc_speed_gain": 1.0,
    "lc_keep_right": 1.0,
    "lc_look_ahead_left": 2.0,
    "lc_speed_gain_right": 1.0,
    "lc_sublane": 1.0,
    "lc_pushy": 0,
    "lc_pushy_gap": 0.6,
    "lc_assertive": 1,
    "lc_accel_lat": 1.0
}

# values to sweep through for some lane change parameters
sweep_dict = OrderedDict({
    "lc_strategic": [1.0, 2.0, 4.0, 8.0],
    "lc_cooperative": [1.0, 2.0],
    "lc_look_ahead_left": [2.0, 4.0]
})

# Create a list of possible lane change parameter combinations.
all_names = sorted(sweep_dict)
combinations = itertools.product(*(sweep_dict[name] for name in all_names))
combination_list = list(combinations)
res = []
for val in combination_list:
    curr_dict = {}
    for elem, name in zip(val, all_names):
        curr_dict[name] = elem
    res.append(curr_dict)

# Create a list of all possible flow_params dictionaries to sweep through the
# different lane change parameters.
flow_params = []

for lane_change_dict in res:
    # no vehicles in the network. The lane change parameters of inflowing
    # vehicles are updated here.
    vehicles = VehicleParams()
    update_dict = deepcopy(default_dict)
    update_dict.update(lane_change_dict)
    vehicles.add(
        "human",
        num_vehicles=0,
        lane_change_params=SumoLaneChangeParams(**update_dict)
    )

    inflow = InFlows()
    # main highway
    inflow.add(
        veh_type="human",
        edge="119257914",
        vehs_per_hour=8378,
        # probability=1.0,
        departLane="random",
        departSpeed=20)
    # on ramp
    inflow.add(
        veh_type="human",
        edge="27414345",
        vehs_per_hour=321,
        departLane="random",
        departSpeed=20)
    inflow.add(
        veh_type="human",
        edge="27414342#0",
        vehs_per_hour=421,
        departLane="random",
        departSpeed=20)

    NET_TEMPLATE = os.path.join(
        config.PROJECT_PATH,
        "examples/exp_configs/templates/sumo/test2.net.xml")

    params = dict(
        # name of the experiment
        exp_tag='I-210_subnetwork',

        # name of the flow environment the experiment is running on
        env_name=TestEnv,

        # name of the network class the experiment is running on
        network=I210SubNetwork,

        # simulator that is used by the experiment
        simulator='traci',

        # simulation-related parameters
        sim=SumoParams(
            sim_step=0.8,
            render=True,
            color_by_speed=True
        ),

        # environment related parameters (see flow.core.params.EnvParams)
        env=EnvParams(
            horizon=4500,  # one hour of run time
        ),

        # network-related parameters (see flow.core.params.NetParams and the
        # network's documentation or ADDITIONAL_NET_PARAMS component)
        net=NetParams(
            inflows=inflow,
            template=NET_TEMPLATE
        ),

        # vehicles to be placed in the network at the start of a rollout (see
        # flow.core.params.VehicleParams)
        veh=vehicles,

        # parameters specifying the positioning of vehicles upon
        # initialization/reset (see flow.core.params.InitialConfig)
        initial=InitialConfig(
            edges_distribution=EDGES_DISTRIBUTION,
        ),
    )

    # Store the next flow_params dict.
    flow_params.append(params)


custom_callables = {
    "avg_merge_speed": lambda env: np.mean(env.k.vehicle.get_speed(
         env.k.vehicle.get_ids_by_edge("119257908#1-AddedOnRampEdge")))
}
