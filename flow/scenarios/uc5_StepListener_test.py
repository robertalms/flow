from __future__ import absolute_import
from __future__ import print_function
import sys
try:
    sys.path.append('/home/robert/sumo/tools/')
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")
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
from flow.core.params import InitialConfig
from flow.core.params import TrafficLightParams

# the Experiment class is used for running simulations
from flow.core.experiment import Experiment

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
    
initial_config = InitialConfig(
    edges_distribution=["e0"]
)
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
debug=False
    
class BaselineRunnerScenario(Scenario):
    
    def __init__(self,
                 downwardEdgeID, 
                 distance, 
                 tocInfos,name, 
                 vehicles, 
                 net_params,
                 initial_config=InitialConfig(),
                 traffic_lights=TrafficLightParams()):
        
        self.downwardEdgeID=downwardEdgeID
        self.distance=distance
        self.tocInfos=tocInfos
        self.downwardToCPending = set()
        self.downwardToCRequested = set()
        
        super().__init__(name, vehicles, net_params, initial_config,traffic_lights)
        
    def getIdentifier(self,fullID, identifierList):
        #~ print ("getIdentifier(%s, %s)"%(fullID, identifierList))
        for start_str in identifierList:
            if fullID.startswith(start_str):
                #~ print("found %s"%start_str)
                return start_str

    def initToCs(self,vehSet, handledSet, edgeID, distance,connection):
        ''' For all vehicles in the given set, check whether they passed the cross section, where a ToC should be triggered and trigger in case. 
        '''
        newTORs = []
        for vehID in vehSet:
            distToTOR = connection.vehicle.getDrivingDistance(vehID, edgeID, distance)
            if distToTOR < 0.:
                handledSet.add(vehID)
                ToCVehicleType = self.getIdentifier(vehID, ToC_lead_times.keys())
                newTORs.append(vehID)
                if ToCVehicleType is not None:
                    # Request a ToC for the vehicle
                    self.requestToC(vehID, ToC_lead_times[ToCVehicleType],connection)
                    if debug:
                        t = connection.simulation.getCurrentTime() / 1000.
                        print("## Requesting ToC for vehicle '%s'!" % (vehID))
                        print("Requested ToC of %s at t=%s (until t=%s)" % (vehID, t, t + float(ToC_lead_times[ToCVehicleType])))
                        self.printToCParams(vehID,connection,True)
                    continue

        return newTORs

    def outputNoToC(self,t, vehIDs, tocInfos):
        for vehID in vehIDs:
            el = ET.Element("noToC")
            el.set("time", str(t))
            el.set("vehID", vehID)
            tocInfos.append(el)


    def outputTORs(self,t, vehIDs, tocInfos,connection):
        for vehID in vehIDs:
            el = ET.Element("TOR")
            el.set("time", str(t))
            el.set("vehID", vehID)
            lastRouteEdgeID = connection.vehicle.getRoute(vehID)[-1]
            lastRouteEdgeLength = connection.lane.getLength(lastRouteEdgeID+"_0")
            distTillRouteEnd = str(connection.vehicle.getDrivingDistance(vehID, lastRouteEdgeID, lastRouteEdgeLength))
            el.set("remainingDist", distTillRouteEnd)
            tocInfos.append(el)

    def outputToCs(self,t, vehIDs, tocInfos,connection):
        for vehID in vehIDs:
            el = ET.Element("ToC")
            el.set("time", str(t))
            el.set("vehID", vehID)
            lastRouteEdgeID = connection.vehicle.getRoute(vehID)[-1]
            lastRouteEdgeLength = connection.lane.getLength(lastRouteEdgeID+"_0")
            distTillRouteEnd = str(connection.vehicle.getDrivingDistance(vehID, lastRouteEdgeID, lastRouteEdgeLength))
            el.set("remainingDist", distTillRouteEnd)
            ToCState = connection.vehicle.getParameter(vehID, "device.toc.state")
            if ToCState != "MRM":
                # vehicle is not performing an MRM
                el.set("MRM", str(0.))
            else:
                # vehicle was performing an MRM, determine the time for which it performed the MRM
                ToCVehicleType = self.getIdentifier(vehID, ToC_lead_times.keys())
                leadTime = ToC_lead_times[ToCVehicleType]
                MRMDuration = float(connection.vehicle.getParameter(vehID, "device.toc.responseTime")) - leadTime
                el.set("MRM", str(MRMDuration))
            tocInfos.append(el)

    def requestToC(self,vehID, timeUntilMRM,connection):
        connection.vehicle.setParameter(vehID, "device.toc.requestToC", str(timeUntilMRM))


    def printToCParams(self,vehID,connection, only_dynamic=False):
        holder = connection.vehicle.getParameter(vehID, "device.toc.holder")
        manualType = connection.vehicle.getParameter(vehID, "device.toc.manualType")
        automatedType = connection.vehicle.getParameter(vehID, "device.toc.automatedType")
        responseTime = connection.vehicle.getParameter(vehID, "device.toc.responseTime")
        recoveryRate = connection.vehicle.getParameter(vehID, "device.toc.recoveryRate")
        initialAwareness = connection.vehicle.getParameter(vehID, "device.toc.initialAwareness")
        mrmDecel = connection.vehicle.getParameter(vehID, "device.toc.mrmDecel")
        currentAwareness = connection.vehicle.getParameter(vehID, "device.toc.currentAwareness")
        state = connection.vehicle.getParameter(vehID, "device.toc.state")
        speed = connection.vehicle.getSpeed(vehID)

        print("time step %s" % connection.simulation.getCurrentTime())
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
class BaselineStepListenerMK(traci.StepListener):
    def __init__(self, scenario):
        self.scenario = scenario
        self.connection = None
    
    def get_connection(self, conn):
        self.connection = conn
    
    def step(self, t):
        if debug:
            print('current sim step %s' % t)
        arrivedVehs = [vehID for vehID in self.connection.simulation.getArrivedIDList() if self.scenario.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
        self.scenario.downwardToCRequested.difference_update(arrivedVehs)
        self.scenario.downwardToCPending.difference_update(arrivedVehs)
        departedToCVehs = [vehID for vehID in self.connection.simulation.getDepartedIDList() if self.scenario.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
        noToC = []
        for vehID in departedToCVehs:
            # set ToC vehicle class to custom1
            #~ print("Departed ToC vehicle '%s'"%vehID)
            if (random.random() < ToCprobability):
                # This vehicle has to perform a ToC
                self.connection.vehicle.setVehicleClass(vehID, "custom1")
            else:
                # vehicle will manage situation witout a ToC
                noToC.append(vehID)
    
        departedToCVehs = [vehID for vehID in departedToCVehs if vehID not in noToC]
        self.scenario.downwardToCPending.update(departedToCVehs)
    
        # provide the ToCService at the specified cross section for informing the lane closure
        newTORs = self.scenario.initToCs(self.scenario.downwardToCPending, self.scenario.downwardToCRequested, downwardEdgeID, distance, self.connection)
        self.scenario.downwardToCPending.difference_update(self.scenario.downwardToCRequested)
    
        # keep book on performed ToCs and trigger best lanes update by resetting the route
        downwardToCPerformed = set()
        for vehID in self.scenario.downwardToCRequested:
            if self.connection.vehicle.getVehicleClass(vehID) == "passenger":
                if debug:
                    print("Downward transition completed for vehicle '%s'" % vehID)
                downwardToCPerformed.add(vehID)
                self.connection.vehicle.updateBestLanes(vehID)
        self.scenario.downwardToCRequested.difference_update(downwardToCPerformed)
    
        # add xml output
        self.scenario.outputNoToC(t, noToC, tocInfos.getroot())
        self.scenario.outputTORs(t, newTORs, tocInfos.getroot(),self.connection)
        self.scenario.outputToCs(t, downwardToCPerformed, tocInfos.getroot(),self.connection)
    
        if debug:
            print("downwardToCRequested=%s" % self.scenario.downwardToCRequested)
            print("DownwardToCPending:%s" % str(sorted(self.scenario.downwardToCPending)))
            print("Length of DownwardToCPending:%s" % str(len(self.scenario.downwardToCPending)))
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
    traci.setConnectHook(myListener.get_connection)

    # create the environment
    env = uc5_env(
        env_params=env_params,
        sim_params=sim_params,
        scenario=baselineScenario,
        simulator='traci',
        stepListener=myListener
    )
    
    # run the simulation for x steps e.g. 36000 --> step == 0.1sec
    exp = Experiment(env=env)
    _ = exp.run(1, 36000)