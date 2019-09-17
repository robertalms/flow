from __future__ import absolute_import
from __future__ import print_function
import sys
try:
    sys.path.append('/home/robert/sumo/tools/')
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import xml.etree.ElementTree as ET
import os
from flow.scenarios import Scenario

from flow.core.params import VehicleParams
from flow.core.params import NetParams
from flow.core.params import InitialConfig
from flow.core.params import EnvParams
from flow.core.params import SumoParams
from flow.core.params import TrafficLightParams

UC5_dir = "/home/robert/flow/examples/UC5/"
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
# # Map from vehicle ID start-string to identify vehicles, that will perform a ToC in the scenario, 
# #     to ToCLead time (in secs) for the corresponding vehicles 
# initial_config = InitialConfig(
#     edges_distribution=["e0"]
# )

downwardEdgeID = None
distance = None
tocInfos=None
ToC_lead_times = {"CAVToC.":10.0, "CVToC.":0.0}
debug=False

class UC5_scenario(Scenario):
    
    def __init__(self,
                 name, 
                 vehicles, 
                 net_params,
                 initial_config=InitialConfig(),
                 traffic_lights=TrafficLightParams()):
        
        self.downwardEdgeID=initial_config.additional_params["downwardEdgeID"]
        self.distance=initial_config.additional_params["distance"]
        self.tocInfos=initial_config.additional_params["tocInfos"]
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