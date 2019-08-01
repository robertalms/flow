# the TestEnv environment is used to simply simulate the network
from flow.envs import TestEnv

# the Experiment class is used for running simulations
from flow.core.experiment import Experiment

# the base scenario class
from flow.scenarios import Scenario

# all other imports are standard
from flow.core.params import VehicleParams
from flow.core.params import NetParams
from flow.core.params import InitialConfig
from flow.core.params import EnvParams
from flow.core.params import SumoParams
import os
os.environ['SUMO_HOME']="/usr/share/sumo"

# create some default parameters parameters
env_params = EnvParams()
initial_config = InitialConfig()
vehicles = VehicleParams()

UC5_dir = "/home/robert/flow/examples/UC5/"


sim_params = SumoParams(render=True, sim_step=0.1)
net_params = NetParams(
      template={
        # network geometry features
        "net": os.path.join(UC5_dir, "UC5_1.net.xml"),
#         "vtype":os.path.join(UC5_dir, "vTypesLV_OS.add.xml")
        "vtype":[os.path.join(UC5_dir, "vTypesLV_OS.add.xml"),
                 os.path.join(UC5_dir, "vTypesCVToC_OS.add.xml"),
                 os.path.join(UC5_dir, "vTypesCAVToC_OS.add.xml")
                ]
    },
    no_internal_links=True
)
    
initial_config = InitialConfig(
    edges_distribution=["e0"]
)
vehicles = VehicleParams()
# vehicles.add('LV', num_vehicles=100)
# vehicles.add('CV', num_vehicles=100)

if __name__ == '__main__':
    
# create the scenario
    scenario = Scenario(
        name="template",
        net_params=net_params,
        initial_config=initial_config,
        vehicles=vehicles
    )
    
    # create the environment
    env = TestEnv(
        env_params=env_params,
        sim_params=sim_params,
        scenario=scenario,
        simulator='newtraci'
    )
    
    # run the simulation for 1000 steps
    exp = Experiment(env=env)
    _ = exp.run(1, 3600)