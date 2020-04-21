#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @author  Leonhard Luecken
# @date    2009-03-26
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import time
import random
import optparse
import xml.etree.ElementTree as ET
# we need to import python modules from the $SUMO_HOME/tools directory
# try:
#     sys.path.append('/usr/share/sumo/tools/')
#     from sumolib import checkBinary  # noqa
# except ImportError:
#     sys.exit(
#         "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
from flow.networks import Network
from flow.core.params import InitialConfig
from flow.core.params import TrafficLightParams

# Vehicle ID start-string to identify vehicles that are informed by TransAID (none for baseline)
AV_identifiers = [] 
# Map from vehicle ID start-string to identify vehicles, that will perform a ToC in the scenario, 
#     to ToCLead time (in secs) for the corresponding vehicles 
ToC_lead_times = {"CAVToC.":10.0, "CVToC.":0.0}
# Probability, that a given CV or CAV has to perform a ToC
# ToCprobability = 0.75
ToCprobability = 1.0
global options
debug=False

class BaselineRunnerScenario(Network):
    
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

    def initToCs(self,vehSet, handledSet, edgeID, distance):
        ''' For all vehicles in the given set, check whether they passed the cross section, where a ToC should be triggered and trigger in case. 
        '''
        global options
        newTORs = []
        for vehID in vehSet:
            distToTOR = traci.vehicle.getDrivingDistance(vehID, edgeID, distance)
            if distToTOR < 0.:
                handledSet.add(vehID)
                ToCVehicleType = self.getIdentifier(vehID, ToC_lead_times.keys())
                newTORs.append(vehID)
                if ToCVehicleType is not None:
                    # Request a ToC for the vehicle
                    self.requestToC(vehID, ToC_lead_times[ToCVehicleType])
                    if debug:
                        t = traci.simulation.getCurrentTime() / 1000.
                        print("## Requesting ToC for vehicle '%s'!" % (vehID))
                        print("Requested ToC of %s at t=%s (until t=%s)" % (vehID, t, t + float(ToC_lead_times[ToCVehicleType])))
                        printToCParams(vehID, True)
                    continue

                # No ToC vehicle
                # TODO: In non-baseline case add a check for the transAID-effectedness (see AV_identifiers)
                #~ TransAIDAffectedType = getIdentifier(vehID, AV_identifiers)
                #~ if TransAIDAffectedType is not None:
                    #~ if options.verbose:
                        #~ print("## Informing AV '%s' about alternative path!" % (vehID))
                    #~ # vClass change indicates that vehicle may run on restricted lane (UC1), which prohibits vClass custom1
                    #~ traci.vehicle.setVehicleClass(vehID, "passenger")
        return newTORs


    def outputNoToC(self,t, vehIDs, tocInfos):
        for vehID in vehIDs:
            el = ET.Element("noToC")
            el.set("time", str(t))
            el.set("vehID", vehID)
            tocInfos.append(el)


    def outputTORs(self,t, vehIDs, tocInfos):
        for vehID in vehIDs:
            el = ET.Element("TOR")
            el.set("time", str(t))
            el.set("vehID", vehID)
            lastRouteEdgeID = traci.vehicle.getRoute(vehID)[-1]
            lastRouteEdgeLength = traci.lane.getLength(lastRouteEdgeID+"_0")
            distTillRouteEnd = str(traci.vehicle.getDrivingDistance(vehID, lastRouteEdgeID, lastRouteEdgeLength))
            el.set("remainingDist", distTillRouteEnd)
            tocInfos.append(el)

    def outputToCs(self,t, vehIDs, tocInfos):
        for vehID in vehIDs:
            el = ET.Element("ToC")
            el.set("time", str(t))
            el.set("vehID", vehID)
            lastRouteEdgeID = traci.vehicle.getRoute(vehID)[-1]
            lastRouteEdgeLength = traci.lane.getLength(lastRouteEdgeID+"_0")
            distTillRouteEnd = str(traci.vehicle.getDrivingDistance(vehID, lastRouteEdgeID, lastRouteEdgeLength))
            el.set("remainingDist", distTillRouteEnd)
            ToCState = traci.vehicle.getParameter(vehID, "device.toc.state")
            if ToCState != "MRM":
                # vehicle is not performing an MRM
                el.set("MRM", str(0.))
            else:
                # vehicle was performing an MRM, determine the time for which it performed the MRM
                ToCVehicleType = self.getIdentifier(vehID, ToC_lead_times.keys())
                leadTime = ToC_lead_times[ToCVehicleType]
                MRMDuration = float(traci.vehicle.getParameter(vehID, "device.toc.responseTime")) - leadTime
                el.set("MRM", str(MRMDuration))
            tocInfos.append(el)

    def run(self,downwardEdgeID, distance, tocInfos): #, upwardEdgeID, upwardDist):
        """execute the TraCI control loop
           tocInfos is an xml element tree to be used to output infos on the toc-processes.
        """
        # this is the list of vehicle states for the scenario, which each AV will traverse
        self.downwardToCPending = set(traci.vehicle.getIDList())
        self.downwardToCRequested = set()
        #~ downwardTocPerformed = set()
        #~ upwardToCPending = set()
        step = 0
        while traci.simulation.getMinExpectedNumber() > 0:
            
            traci.simulationStep()
            t = traci.simulation.getCurrentTime()/1000.
            step += 1
#             if debug:
#                 print("\n---------------------------------\nsimstep: %s" % step)
#             # Keep book of entered AVs
#             arrivedVehs = [vehID for vehID in traci.simulation.getArrivedIDList() if self.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
#             downwardToCRequested.difference_update(arrivedVehs)
#             downwardToCPending.difference_update(arrivedVehs)
#             departedToCVehs = [vehID for vehID in traci.simulation.getDepartedIDList() if self.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
#             #~ if (len(traci.simulation.getDepartedIDList())>0):
#                 #~ print("departed = %s"%traci.simulation.getDepartedIDList())
#                 #~ print("departedToCVehs = %s"%departedToCVehs)
#             noToC = []
#             for vehID in departedToCVehs:
#                 # set ToC vehicle class to custom1
#                 #~ print("Departed ToC vehicle '%s'"%vehID)
#                 if (random.random() < ToCprobability):
#                     # This vehicle has to perform a ToC
#                     traci.vehicle.setVehicleClass(vehID, "custom1")
#                 else:
#                     # vehicle will manage situation witout a ToC
#                     noToC.append(vehID)
# 
#             departedToCVehs = [vehID for vehID in departedToCVehs if vehID not in noToC]
#             downwardToCPending.update(departedToCVehs)
# 
#             # provide the ToCService at the specified cross section for informing the lane closure
#             newTORs = self.initToCs(downwardToCPending, downwardToCRequested, downwardEdgeID, distance)
#             downwardToCPending.difference_update(downwardToCRequested)
# 
#             # keep book on performed ToCs and trigger best lanes update by resetting the route
#             downwardToCPerformed = set()
#             for vehID in downwardToCRequested:
#                 if traci.vehicle.getVehicleClass(vehID) == "passenger":
#                     if debug:
#                         print("Downward transition completed for vehicle '%s'" % vehID)
#                     downwardToCPerformed.add(vehID)
#                     traci.vehicle.updateBestLanes(vehID)
#             downwardToCRequested.difference_update(downwardToCPerformed)
#             #~ upwardToCPending.update(downwardTocPerformed) # no upwards ToC for baseline
# 
#             # add xml output
#             self.outputNoToC(t, noToC, tocInfos.getroot())
#             self.outputTORs(t, newTORs, tocInfos.getroot())
#             self.outputToCs(t, downwardToCPerformed, tocInfos.getroot())
# 
#             #~ # provide ToCService to the upwardTransitions
#             #~ upwardTocPerformed = set()
#             #~ doToC(upwardToCPending, upwardTocPerformed, 0., upwardEdgeID, upwardDist)
#             #~ upwardToCPending.difference_update(upwardTocPerformed)
#             if debug:
#                 print("downwardToCRequested=%s" % downwardToCRequested)
#                 #~ print("Downward ToC performed: %s" % str(sorted(downwardTocPerformed)))
#                 #~ print("Upward ToC performed: %s" % str(sorted(upwardTocPerformed)))
#                 print("DownwardToCPending:%s" % str(sorted(downwardToCPending)))
#                 #~ print("upwardToCPending:%s" % str(sorted(upwardToCPending)))
#                 #~ upwardToCPending.difference_update(arrivedVehs)


    def requestToC(self,vehID, timeUntilMRM):
        traci.vehicle.setParameter(vehID, "device.toc.requestToC", str(timeUntilMRM))


    def printToCParams(self,vehID, only_dynamic=False):
        holder = traci.vehicle.getParameter(vehID, "device.toc.holder")
        manualType = traci.vehicle.getParameter(vehID, "device.toc.manualType")
        automatedType = traci.vehicle.getParameter(vehID, "device.toc.automatedType")
        responseTime = traci.vehicle.getParameter(vehID, "device.toc.responseTime")
        recoveryRate = traci.vehicle.getParameter(vehID, "device.toc.recoveryRate")
        initialAwareness = traci.vehicle.getParameter(vehID, "device.toc.initialAwareness")
        mrmDecel = traci.vehicle.getParameter(vehID, "device.toc.mrmDecel")
        currentAwareness = traci.vehicle.getParameter(vehID, "device.toc.currentAwareness")
        state = traci.vehicle.getParameter(vehID, "device.toc.state")
        speed = traci.vehicle.getSpeed(vehID)

        print("time step %s" % traci.simulation.getCurrentTime())
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

from flow.envs import Env

# the Experiment class is used for running simulations
from flow.core.experiment import Experiment

# the base scenario class
from flow.networks import Network

# all other imports are standard
from flow.core.params import VehicleParams
from flow.core.params import NetParams
from flow.core.params import InitialConfig
from flow.core.params import EnvParams
from flow.core.params import SumoParams
import os
os.environ['SUMO_HOME']="/usr/share/sumo"

# create some default parameters parameters
ADDITIONAL_ENV_PARAMS = {
    "max_accel": 1,
    "max_decel": 1,
}
#env_params = EnvParams()
env_params = EnvParams(additional_params=ADDITIONAL_ENV_PARAMS) #will be used later for TestEnvironment
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
    }#,
#     no_internal_links=True
)
    
initial_config = InitialConfig(
    edges_distribution=["e0"]
)
vehicles = VehicleParams()

# this is the main entry point of this script
#options = get_options()

UC5_dir = "/home/robert/flow/examples/UC5/"
downwardEdgeID = None
distance = None


class BaselineStepListener(traci.StepListener):
    
    def __init__(self, scenario):
        self.scenario = scenario
    
    def step(self, t):
        print('current sim step %s' % t)
        arrivedVehs = [vehID for vehID in traci.simulation.getArrivedIDList() if self.scenario.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
        self.scenario.downwardToCRequested.difference_update(arrivedVehs)
        self.scenario.downwardToCPending.difference_update(arrivedVehs)
        departedToCVehs = [vehID for vehID in traci.simulation.getDepartedIDList() if self.scenario.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
        #~ if (len(traci.simulation.getDepartedIDList())>0):
            #~ print("departed = %s"%traci.simulation.getDepartedIDList())
            #~ print("departedToCVehs = %s"%departedToCVehs)
        noToC = []
        for vehID in departedToCVehs:
            # set ToC vehicle class to custom1
            #~ print("Departed ToC vehicle '%s'"%vehID)
            if (random.random() < ToCprobability):
                # This vehicle has to perform a ToC
                traci.vehicle.setVehicleClass(vehID, "custom1")
            else:
                # vehicle will manage situation witout a ToC
                noToC.append(vehID)
    
        departedToCVehs = [vehID for vehID in departedToCVehs if vehID not in noToC]
        self.scenario.downwardToCPending.update(departedToCVehs)
    
        # provide the ToCService at the specified cross section for informing the lane closure
        newTORs = self.scenario.initToCs(self.scenario.downwardToCPending, self.scenario.downwardToCRequested, downwardEdgeID, distance)
        self.scenario.downwardToCPending.difference_update(self.scenario.downwardToCRequested)
    
        # keep book on performed ToCs and trigger best lanes update by resetting the route
        downwardToCPerformed = set()
        for vehID in self.scenario.downwardToCRequested:
            if traci.vehicle.getVehicleClass(vehID) == "passenger":
                if debug:
                    print("Downward transition completed for vehicle '%s'" % vehID)
                downwardToCPerformed.add(vehID)
                traci.vehicle.updateBestLanes(vehID)
        self.scenario.downwardToCRequested.difference_update(downwardToCPerformed)
        #~ upwardToCPending.update(downwardTocPerformed) # no upwards ToC for baseline
    
        # add xml output
        self.scenario.outputNoToC(t, noToC, tocInfos.getroot())
        self.scenario.outputTORs(t, newTORs, tocInfos.getroot())
        self.scenario.outputToCs(t, downwardToCPerformed, tocInfos.getroot())
    
        #~ # provide ToCService to the upwardTransitions
        #~ upwardTocPerformed = set()
        #~ doToC(upwardToCPending, upwardTocPerformed, 0., upwardEdgeID, upwardDist)
        #~ upwardToCPending.difference_update(upwardTocPerformed)
        if debug:
            print("downwardToCRequested=%s" % downwardToCRequested)
            #~ print("Downward ToC performed: %s" % str(sorted(downwardTocPerformed)))
            #~ print("Upward ToC performed: %s" % str(sorted(upwardTocPerformed)))
            print("DownwardToCPending:%s" % str(sorted(downwardToCPending)))
            #~ print("upwardToCPending:%s" % str(sorted(upwardToCPending)))
            #~ upwardToCPending.difference_update(arrivedVehs)
        return True


if __name__ == '__main__':
        
    traci.start(['sumo-gui',
                 "-c", UC5_dir+"uc5.sumo.cfg", 
                 "-r", UC5_dir+"routes_trafficMix_0_trafficDemand_1_driverBehaviour_OS_seed_0.xml",
                 "-a", "%sadditionalsOutput_trafficMix_0_trafficDemand_1_driverBehaviour_OS_seed_0.xml, %svTypesCAVToC_OS.add.xml, %svTypesCVToC_OS.add.xml, %svTypesLV_OS.add.xml"%(UC5_dir,UC5_dir,UC5_dir,UC5_dir)
    ])
    
    # provide etree for ToC infos
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

    myListener = BaselineStepListener(baselineScenario)
    traci.addStepListener(myListener)
    
    baselineScenario.run(downwardEdgeID, distance, tocInfos)
    
    traci.close()
    tocInfos.write("%soutputToCs"%UC5_dir)
    sys.stdout.flush()