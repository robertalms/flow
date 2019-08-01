from __future__ import absolute_import
from __future__ import print_function
from flow.envs import Env
from gym.spaces.box import Box
from gym.spaces.tuple_space import Tuple
import numpy as np
import os
import sys
import time
import random
import optparse
import xml.etree.ElementTree as ET
# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append('/usr/share/sumo/tools/')
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
from flow.scenarios import Scenario
from flow.core.params import InitialConfig
from flow.core.params import TrafficLightParams

from flow.envs import Env

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
                ]
    }
)
    
initial_config = InitialConfig(
    edges_distribution=["e0"]
)
vehicles = VehicleParams()
##################### Parameters for BaseRunner-Class #############################################
downwardEdgeID = None
distance = None

# Vehicle ID start-string to identify vehicles that are informed by TransAID (none for baseline)
AV_identifiers = [] 
# Map from vehicle ID start-string to identify vehicles, that will perform a ToC in the scenario, 
#     to ToCLead time (in secs) for the corresponding vehicles 
ToC_lead_times = {"CAVToC.":10.0, "CVToC.":0.0}
# Probability, that a given CV or CAV has to perform a ToC
ToCprobability = 1.0
global options
debug=False

# define the environment class myEnv, and inherit properties from the base environment class Env
class myEnv(Env):
    @property
    def action_space(self):
        num_actions = self.initial_vehicles.num_rl_vehicles
        accel_ub = self.env_params.additional_params["max_accel"]
        accel_lb = - abs(self.env_params.additional_params["max_decel"])

        return Box(low=accel_lb,
                   high=accel_ub,
                   shape=(num_actions,))
    @property
    def observation_space(self):
        return Box(
            low=0,
            high=float("inf"),
            shape=(2*self.initial_vehicle.num_vehicles,),
        )
    def _apply_rl_actions(self, rl_actions):
        # the names of all autonomous (RL) vehicles in the network
        rl_ids = self.k.vehicle.get_rl_ids()

        # use the base environment method to convert actions into accelerations for the rl vehicles
        self.k.vehicle.apply_acceleration(rl_ids, rl_actions)
    def get_state(self, **kwargs):
        # the get_ids() method is used to get the names of all vehicles in the network
        ids = self.k.vehicle.get_ids()

        # we use the get_absolute_position method to get the positions of all vehicles
        pos = [self.k.vehicle.get_x_by_id(veh_id) for veh_id in ids]

        # we use the get_speed method to get the velocities of all vehicles
        vel = [self.k.vehicle.get_speed(veh_id) for veh_id in ids]

        # the speeds and positions are concatenated to produce the state
        return np.concatenate((pos, vel))
    def compute_reward(self, rl_actions, **kwargs):
        # the get_ids() method is used to get the names of all vehicles in the network
        ids = self.k.vehicle.get_ids()

        # we next get a list of the speeds of all vehicles in the network
        speeds = self.k.vehicle.get_speed(ids)

        # finally, we return the average of all these speeds as the reward
        return np.mean(speeds) 
###################################################################################################    
class BaselineRunnerScenario(Scenario):
    
    def __init__(self,
                 downwardEdgeID, 
                 distance, 
                 tocInfos,
                 name, 
                 vehicles, 
                 net_params,
                 initial_config=InitialConfig(),
                 traffic_lights=TrafficLightParams()):
        
        self.downwardEdgeID=downwardEdgeID
        self.distance=distance
        self.tocInfos=tocInfos
        self.downwardToCPending = set(vehicles.ids)
        self.downwardToCRequested = set()
        
        super().__init__(name, vehicles, net_params, initial_config,traffic_lights)
        
    def getIdentifier(self,fullID, identifierList):
        #~ print ("getIdentifier(%s, %s)"%(fullID, identifierList))
        for start_str in identifierList:
            if fullID.startswith(start_str):
                #~ print("found %s"%start_str)
                return start_str

    def initToCs(self,vehSet, handledSet, edgeID, distance, master_kernel):
        ''' For all vehicles in the given set, check whether they passed the cross section, where a ToC should be triggered and trigger in case. 
        '''
        global options
        newTORs = []
        for vehID in vehSet:
            distToTOR = master_kernel.vehicle.get_driving_distance(vehID, edgeID, distance)
            if distToTOR < 0.:
                handledSet.add(vehID)
                ToCVehicleType = self.getIdentifier(vehID, ToC_lead_times.keys())
                newTORs.append(vehID)
                if ToCVehicleType is not None:
                    # Request a ToC for the vehicle
                    self.requestToC(vehID, ToC_lead_times[ToCVehicleType], master_kernel)
                    if debug:
                        t = master_kernel.simulation.kernel_api.simulation.getTime()
                        print("## Requesting ToC for vehicle '%s'!" % (vehID))
                        print("Requested ToC of %s at t=%s (until t=%s)" % (vehID, t, t + float(ToC_lead_times[ToCVehicleType])))
                        self.printToCParams(vehID, master_kernel, False)
                    continue

        return newTORs

    def outputNoToC(self,t, vehIDs, tocInfos):
        for vehID in vehIDs:
            el = ET.Element("noToC")
            el.set("time", str(t))
            el.set("vehID", vehID)
            tocInfos.append(el)


    def outputTORs(self,t, vehIDs, tocInfos,master_kernel):
        for vehID in vehIDs:
            el = ET.Element("TOR")
            el.set("time", str(t))
            el.set("vehID", vehID)
            lastRouteEdgeID = master_kernel.vehicle.get_route(vehID)[-1]
            lastRouteEdgeLength = master_kernel.vehicle.get_lane_length(lastRouteEdgeID+"_0")
            distTillRouteEnd = str(master_kernel.vehicle.get_driving_distance(vehID, lastRouteEdgeID, lastRouteEdgeLength))
            el.set("remainingDist", distTillRouteEnd)
            tocInfos.append(el)

    def outputToCs(self,t, vehIDs, tocInfos,master_kernel):
        for vehID in vehIDs:
            el = ET.Element("ToC")
            el.set("time", str(t))
            el.set("vehID", vehID)
            lastRouteEdgeID = master_kernel.vehicle.get_route(vehID)[-1]
            lastRouteEdgeLength = master_kernel.vehicle.get_lane_length(lastRouteEdgeID+"_0")
            distTillRouteEnd = str(master_kernel.vehicle.get_driving_distance(vehID, lastRouteEdgeID, lastRouteEdgeLength))
            el.set("remainingDist", distTillRouteEnd)
            ToCState = master_kernel.vehicle.get_parameter(vehID, "device.toc.state")
            if ToCState != "MRM":
                # vehicle is not performing an MRM
                el.set("MRM", str(0.))
            else:
                # vehicle was performing an MRM, determine the time for which it performed the MRM
                ToCVehicleType = self.getIdentifier(vehID, ToC_lead_times.keys())
                leadTime = ToC_lead_times[ToCVehicleType]
                MRMDuration = float(master_kernel.vehicle.get_parameter(vehID, "device.toc.responseTime")) - leadTime
                el.set("MRM", str(MRMDuration))
            tocInfos.append(el)

    def requestToC(self,vehID, timeUntilMRM, master_kernel):
        master_kernel.vehicle.set_parameter(vehID, "device.toc.requestToC", str(timeUntilMRM))

    def printToCParams(self,vehID, master_kernel, only_dynamic=False):
        holder = master_kernel.vehicle.get_parameter(vehID, "device.toc.holder")
        manualType = master_kernel.vehicle.get_parameter(vehID, "device.toc.manualType")
        automatedType = master_kernel.vehicle.get_parameter(vehID, "device.toc.automatedType")
        responseTime = master_kernel.vehicle.get_parameter(vehID, "device.toc.responseTime")
        recoveryRate = master_kernel.vehicle.get_parameter(vehID, "device.toc.recoveryRate")
        initialAwareness = master_kernel.vehicle.get_parameter(vehID, "device.toc.initialAwareness")
        mrmDecel = master_kernel.vehicle.get_parameter(vehID, "device.toc.mrmDecel")
        currentAwareness = master_kernel.vehicle.get_parameter(vehID, "device.toc.currentAwareness")
        state = master_kernel.vehicle.get_parameter(vehID, "device.toc.state")
        speed = master_kernel.vehicle.get_speed(vehID)
 
        print("time step %s" % master_kernel.simulation.kernel_api.simulation.getTime())
        print("ToC device infos for vehicle '%s'" % vehID)
        if not only_dynamic:
            print("Static parameters:")
            print("  holder = %s" % holder)
            print("  manualType = %s" % manualType)
            print("  automatedType = %s" % automatedType)
            print("  responseTime = %s" % responseTime)
            print("  recoveryRate = %s" % recoveryRate)
            print("  initialAwareness = %s" % initialAwareness)
            print("  mrmDecel = %s" % mrmDecel)
            print("Dynamic parameters:")
        print("  currentAwareness = %s" % currentAwareness)
        print("  currentSpeed = %s" % speed)
        print("  state = %s" % state)
###################################################################################################

class BaselineStepListenerMK():
    def __init__(self, scenario):
        self.scenario = scenario
        
    def step(self, master_kernel, t):
        if debug:
            print('current sim step %s' % t)
        arrivedVehs = [vehID for vehID in master_kernel.vehicle.get_arrived_ids() if self.scenario.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
        self.scenario.downwardToCRequested.difference_update(arrivedVehs)
        self.scenario.downwardToCPending.difference_update(arrivedVehs)
        departedToCVehs = [vehID for vehID in master_kernel.vehicle.get_departed_ids() if self.scenario.getIdentifier(vehID, ToC_lead_times.keys()) is not None]

        noToC = []
        for vehID in departedToCVehs:
            # set ToC vehicle class to custom1
            #~ print("Departed ToC vehicle '%s'"%vehID)
            if (random.random() < ToCprobability):
                # This vehicle has to perform a ToC
                master_kernel.vehicle.set_vehicle_class(vehID, "custom1")
            else:
                # vehicle will manage situation witout a ToC
                noToC.append(vehID)
    
        departedToCVehs = [vehID for vehID in departedToCVehs if vehID not in noToC]
        self.scenario.downwardToCPending.update(departedToCVehs)
    
        # provide the ToCService at the specified cross section for informing the lane closure
        newTORs = self.scenario.initToCs(self.scenario.downwardToCPending, self.scenario.downwardToCRequested, downwardEdgeID, distance, master_kernel)
        self.scenario.downwardToCPending.difference_update(self.scenario.downwardToCRequested)
    
        # keep book on performed ToCs and trigger best lanes update by resetting the route
        downwardToCPerformed = set()
        for vehID in self.scenario.downwardToCRequested:
            if master_kernel.vehicle.get_vehicle_class(vehID) == "passenger":
                if debug:
                    print("Downward transition completed for vehicle '%s'" % vehID)
                    self.scenario.printToCParams(vehID, master_kernel, False)
                downwardToCPerformed.add(vehID)
                master_kernel.vehicle.update_best_lanes(vehID)
        self.scenario.downwardToCRequested.difference_update(downwardToCPerformed)
    
        # add xml output
        self.scenario.outputNoToC(t, noToC, tocInfos.getroot())
        self.scenario.outputTORs(t, newTORs, tocInfos.getroot(),master_kernel)
        self.scenario.outputToCs(t, downwardToCPerformed, tocInfos.getroot(),master_kernel)
    
        if debug:
            print("downwardToCRequested=%s" % self.scenario.downwardToCRequested)
            print("DownwardToCPending:%s" % str(sorted(self.scenario.downwardToCPending)))

        return True
###################################################################################################

if __name__ == '__main__':
    tocInfos = ET.ElementTree(ET.Element("tocInfos"))

    downwardEdgeID = "e0"
    distance = 2300. 
    
    baselineScenario = BaselineRunnerScenario(downwardEdgeID, 
                           distance, 
                           tocInfos,
                           name="template",
                           net_params=net_params,
                           initial_config=initial_config,
                           vehicles=vehicles
                          ) 
    
    myListener = BaselineStepListenerMK(baselineScenario)
  
    # create the environment
    env = myEnv(
        env_params=env_params,
        sim_params=sim_params,
        scenario=baselineScenario,
        simulator='newtraci',
        stepListener=[myListener]
    )
    
    # run the simulation for x steps e.g. 36000 --> step == 0.1sec
    exp = Experiment(env=env)
    _ = exp.run(1, 36000)