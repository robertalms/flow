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

import sys
import random
import optparse
import math
from warnings import warn

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append('/usr/share/sumo/tools/')
#     sys.path.append(os.path.join(os.path.dirname(
#         __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
#     sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
#         os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
import traci.constants as tc

# Vehicle ID start-string to identify vehicles that are informed by TransAID (none for baseline)
AV_identifiers = [] 
# Map from vehicle ID start-string to identify vehicles, that will perform a ToC in the scenario, 
#     to ToCLead time (in secs) for the corresponding vehicles 
ToC_lead_times = {"CAVToC.":10.0, "CVToC.":0.0, "LV.":-1.0}
# Probability, that a given CV or CAV has to perform a ToC
ToCprobability = 1.0
global options

## Parameters for platoon management
# Spacing and time gap (thresholds for accepting new vehicles)
# (Spacing and timegap operate conjunctively: if one encopasses the candidate 
#  vehicle, it can be added) @see checkPlatoonMembership()
MAX_PLATOON_SPACING = 20
MAX_PLATOON_TIMEGAP = 3.5
# Distance beyond the ToC zone entry, at which an entering platoon will be closed the latest
# (heuristic value: road max speed * max time gap = 3.5 * 36.11 ~ 120 m)
PLATOON_CLOSING_DIST = 120
# Closing time is the maximal time after which a platoon is closed if no further vehicles are added
PLATOON_CLOSING_TIME = 5

## Network configuration
# This script assumes that there is a single edge ('e0') on which the ToCs are taking place. 
# Entry position of the NoAD zone on edge e0
NOAD_ZONE_ENTRY_POS = 2300.

## ToC parameter constants
# Maximal time until MRM is imposed in case of driver irresponsiveness
TIME_TILL_MRM = ToC_lead_times["CAVToC."]
# Minimal deceleration rate applied during MRM 
MRM_DECEL = 3.0

## Parameter for the calculation of the inter-TOR interval used for 
#  disengaging a platoon vehicle by vehicle, @see calculateTORInterval()
# Assumed maximal brake rate applied during opening a gap in the preparation phase for a takeover
OPENGAP_BRAKE_RATE = 1.0
# Assumed minimal spacing to be obtained by the open gap mechanism
OPENGAP_SPACING = 10
# Assumed minimal time headway to be obtained by the open gap mechanism
OPENGAP_TIMEGAP = 2.0
# Assumed minimal time gap used by the automated car-following controller
AUTOMATED_TIMEGAP = 1.5
# Value for the lane occupancy (in %), at which TORs should be issued immediately at vehicle detection
MAX_OCCUPANCY = 10. 


def drawRandomColor(seed):
    random.seed(seed)
    c = random.randint(0,256*256*256-1)
    r = c%256
    g = int((c/256)%256)
    b = int((c/(256*256))%256)
    return (r,g,b)    

class Platoon:
    _nextPlatoonID = 0;
    def __init__(self, ID, veh):
        print("Creating new platoon '%s' with leading vehicle '%s'"%(ID, veh.ID))
        # vehs is a map: vehID -> (platoon position index, Vehicle object)
        self.vehs={veh.ID : [0, veh]}
        self.trailer = veh
        self.leader = veh
        self.ID = ID
        self.isClosed = False
        self.color = drawRandomColor(ID)
        traci.vehicle.setColor(veh.ID, self.color)
    def appendVeh(self,veh):
        print("Appending vehicle '%s' to platoon '%s'"%(veh.ID, self.ID))
        self.vehs[veh.ID] = (len(self.vehs), veh)
        self.trailer = veh
        traci.vehicle.setColor(veh.ID, self.color)
    def close(self):
        self.isClosed = True
    def removeVehicles(self, vehIDList):
        ''' removeVehicles(list(string)) -> list(Vehicle)
        Removes vehicles with the given IDs from the platoon.
        '''
        print("Removing vehicles from platoon '%s': %s"%(self.ID, str(vehIDList)))
        removed = []
        for vehID in vehIDList:
            veh = self.vehs.pop(vehID, None)
            if veh:
                veh = veh[1]
                traci.vehicle.setColor(veh.ID, veh.origColor)
                removed.append(veh)
        self._updatePositionIndices()
        return removed
        
    def _updatePositionIndices(self):
        '''_updatePositionIndices()
        Ensures position indices to range from 0 to len(vehs)-1 without reordering the vehicles
        '''
        if(len(self.vehs) == 0):
            return       
        # Rebuild vehs sorted according to index
        sortedVehs = [veh for (_,veh) in sorted(self.vehs.values())]
        self.vehs = dict([(veh.ID, (i, veh)) for i, veh in enumerate(sortedVehs)])
        # update trailer and leader
        self.leader = sortedVehs[0]
        self.trailer = sortedVehs[-1]
        
    
    def getTrailer(self):
        ''' getTrailer() -> veh
        Returns last vehicle in platoon. (Assumes constant order -> trailer is always the last appended vehicle)
        '''
        return self.trailer
    def getLeader(self):
        ''' getLeader() -> veh
        Returns first vehicle in platoon. (Assumes constant order -> leader is always the vehicle given to the constructor)
        '''
        return self.leader
    def getVehicles(self):
        ''' getVehicles() -> iterable((position index, Vehicle))
        Returns the platoons (position, vehicle)-tuples 
        '''
        return self.vehs.values()
        
class Vehicle:
    def __init__(self, pos, speed, lane, length, minGap, vehID, automationType, detectionTime):
        self.pos=pos
        self.speed=speed
        self.lane=lane
        self.minGap = minGap
        self.length = length
        self.ID=vehID
        self.TORstate='noTOR'
        self.automationType=automationType
        self.detectionTime = detectionTime
        self.xTOR = None
        self.thTOR = None
        self.origColor = traci.vehicle.getColor(vehID)
    def setState(self,TOR):
        self.TORstate=TOR
    def updatePosition(self):
        self.pos=traci.vehicle.getLanePosition(self.ID)
    
    def getID(self):
        return self.ID
    def getAutomationType(self):
        return self.automationType
    def getLane(self):
        return self.lane
    def getPos(self):
        return self.pos
    def getState(self):
        return self.TORstate
           
def getIdentifier(fullID, identifierList):
    #~ print ("getIdentifier(%s, %s)"%(fullID, identifierList))
    for start_str in identifierList:
        if fullID.startswith(start_str):
            #~ print("found %s"%start_str)
            return start_str

def getSpacing(follower, leader):
    ''' getSpacing(Vehicle, Vehicle) -> float
    Returns the positive front to backbumper distance between the two given vehicles if applicable, -1 otherwise.
    '''
    leaderEdgeID = leader.lane[:-2]
    return traci.vehicle.getDrivingDistance(follower.ID, leaderEdgeID, leader.pos - leader.length - follower.minGap)

def checkPlatoonMembership(plt, veh):
    ''' checkPlatoonMembership(Platoon, Vehicle) -> bool
    Determines whether the given vehicle should be added to the given platoon based on the time 
    and space headway between the individual vehicle and the last member of the platoon.
    (The platoon members are assumed to be ordered descendingly in position.) 
    '''
    global MAX_PLATOON_SPACING, MAX_PLATOON_TIMEGAP
    # Can't add to non-existing platoon
    if plt is None:
        return False
    assert(len(plt.vehs) > 0)
    # trailing vehicle in platoon
    lastMember = plt.getTrailer()
    # distance between individual vehicle and the trailer
    dist = getSpacing(veh, lastMember)
    
    # ... should be ahead of the added vehicle
    if(dist < 0):
        return False
    # ... but sufficiently close in space and time 
    elif(dist > max(MAX_PLATOON_SPACING, veh.speed*MAX_PLATOON_TIMEGAP)):
        return False 
    else:
        # Vehicle in platoon distance
        return True
    
def subscribeState(vehID):
    ''' subscribeState(string) -> None
    Adds a traci subscription for the vehicle's state (lanePos, speed, laneID)
    '''
    print("subscribeState() for vehicle '%s'"%vehID)
    traci.vehicle.subscribe(vehID, [tc.VAR_LANE_ID, tc.VAR_LANEPOSITION, tc.VAR_SPEED])
    
def updatePositions(vehicleList):
    ''' updatePositions() -> None
    Updates the lanes and positions for the given lists of vehicles and platoons.
    '''
    print("updatePositions() for vehicles: %s"%str([veh.ID for veh in vehicleList]))
    for veh in vehicleList:
        traciResults = traci.vehicle.getSubscriptionResults(veh.ID)
        #print("traciResults: %s"%str(traciResults))
        veh.pos = traciResults[tc.VAR_LANEPOSITION]
        veh.speed = traciResults[tc.VAR_SPEED]
        veh.lane = traciResults[tc.VAR_LANE_ID]

def calculateTORInterval(speed):
    ''' calculateTORInterval(float) -> float
    This calculates the time interval Dt between successive TORs issued when
    disengaging automations of platoon members vehicle by vehicle starting at the
    last vehicle.
    Within the time Dt a gap of size g1 = OPENGAP_SPACING + OPENGAP_TIMEGAP*speed
    should be established by braking at uniform rate b = OPENGAP_BRAKE_RATE.
    Assumingly, the minimal current time headway can be as small as gA = AUTOMATED_TIMEGAP*speed.
    Thus the desired increase of the gap fulfills Dg = g1-gA = 0.5*Dt*Dt/b.
    This yields Dt = sqrt(2*Dg*b)
    '''
    global OPENGAP_BRAKE_RATE, OPENGAP_SPACING, OPENGAP_TIMEGAP, AUTOMATED_TIMEGAP
    # Desired gap increase
    Dg = OPENGAP_SPACING + (OPENGAP_TIMEGAP - AUTOMATED_TIMEGAP)*speed
    # constant brake rate
    b = OPENGAP_BRAKE_RATE
    # inter-TOR interval
    Dt = math.sqrt(2*Dg*b)
    return Dt
    
def calculateTORPositions(plt, occupancyLevel):
    ''' calculateTORPositions(Platoon, float, float, float) -> None
    Determines the positional TOR-coefficients for the platoon in dependence of the downstream density
    and stores them for the platoon members.   
    For N members, the scheduled TOR times for the vehicles are given as
          t_i = t_0 - i*Dt     (1)
    where t_0 is the TOR time of the leader and Dt are the intervals between TORs given to successive platoon members.
    
    t_0 is determined as the estimated arrival time tMax of the leader at a point between xMax, the point at the closest
    admissible distance to the NoAD zone for a TOR, and its current position. The chosen point scales 
    linearly between these extremes with the downstreamDensity in [0,1]. For a density of one, the leaders
    TOR is issued immediately.
    
    Given t_i, these yield corresponding TOR-points 
        x^i_TOR = x_i + t_i*v0,      (2)
    with the platoon leader's speed v0, the current position x_i of the i-th vehicle. The TOR coefficients are
    given as proportions of distance to x^i_TOR and xTOR:  
        theta_i = (x^i_TOR - x_i)/(xMax - x_i)    (3)
    That is:
        x^i_TOR = x_i + theta_i*(xMax - x_i),    (4)
    which is used to update the value of x^i_TOR when xTOR changes (which it does everytime that v0 changes)
    '''
    global NOAD_ZONE_ENTRY_POS, TIME_TILL_MRM, MRM_DECEL
    print("calculateTORPositions() for platoon '%s' at occupancyLevel %s"%(plt.ID, occupancyLevel))
    
    # Platoon leader
    leader = plt.getLeader()
    # Distance travelled until MRM induced stop 
    nonresponsiveDist = leader.speed*TIME_TILL_MRM + 0.5*leader.speed*leader.speed/MRM_DECEL
    # maximal point at which a TOR needs to be issued to ensure a stop before the NoAD zone
    xMax = NOAD_ZONE_ENTRY_POS - nonresponsiveDist 
    
    # Calculate t_i (stored in T)
    tMax = (xMax - leader.pos)/max(leader.speed, 0.001)
    t0 = min(1.,max(0.,1 - occupancyLevel))*tMax   
    N = len(plt.vehs)
    if N > 1:
        Dt = calculateTORInterval(leader.speed)
        print("Calculated TOR interval for platoon '%s' with v0=%s as Dt=%s"%(plt.ID, leader.speed, Dt))
    else:
        # no need to calculate Dt in case of solitary vehicle 
        Dt = 0
    T = [t0 - i*Dt for i in range(N)]
    
    for i, veh in plt.getVehicles():
        ti = T[i]
        # Calculate x^i_TOR (stored in veh_i.xTOR)
        veh.xTOR = veh.pos + ti*leader.speed
        # Calculate theta_i (stored in veh_i.thTOR)
        veh.thTOR = (veh.xTOR - veh.pos)/(xMax - veh.pos)
        # store start point for vehicle i
        veh.scalingReferencePoint = veh.pos
        print("   %s -> '%s': pos=%s, xTOR=%s, thTOR=%s (x_ref=%s)"%(i, veh.ID, veh.pos, veh.xTOR, veh.thTOR, veh.scalingReferencePoint))
    
    
def updateTORPositions(plt):
    ''' updateTORPositions(Platoon) -> None
    Updates the positions at which a TOR should be issued for the single vehicles in the platoon.
    @see closePlatoon(), eqn.(4)
    '''
    global NOAD_ZONE_ENTRY_POS, TIME_TILL_MRM, MRM_DECEL
    print("updateTORPositions() for platoon '%s'"%plt.ID)
    
    # Platoon leader
    leader = plt.getLeader()
    # Distance travelled until MRM induced stop 
    nonresponsiveDist = leader.speed*TIME_TILL_MRM + 0.5*leader.speed*leader.speed/MRM_DECEL
    # maximal point at which a TOR needs to be issued to ensure a stop before the NoAD zone
    xMax = NOAD_ZONE_ENTRY_POS - nonresponsiveDist

    # Recalculate x^i_TOR (stored in veh_i.xTOR) from theta_i
    for i, veh in plt.getVehicles():
        veh.xTOR = veh.scalingReferencePoint + veh.thTOR*(xMax - veh.scalingReferencePoint)
        print("   %s -> '%s': pos=%s, xTOR=%s, thTOR=%s (x_ref=%s)"%(i, veh.ID, veh.pos, veh.xTOR, veh.thTOR, veh.scalingReferencePoint))
        
        
def issueTORs(plt):
    ''' issueTORs(Platoon) -> None
    Command TORs for all vehicles which passed their TOR positions,
    remove them from platoon, and return them 
    '''
    toRemove = []
    for _, veh in plt.vehs.values():
        if veh.pos >= veh.xTOR:
            print("Vehicle '%s' in platoon '%s' passed xTOR. Requesting ToC."%(veh.ID, plt.ID))
            requestToC(veh.ID, ToC_lead_times[veh.automationType])
            toRemove.append(veh.ID)
    if toRemove:
        return plt.removeVehicles(toRemove)
    else:
        return []
    
def removeVehiclesBeyond(x, vehList):
    ''' removeVehiclesBeyond(float, vehList)
    Removes all vehicles with position > x from the list.
    '''
    toRemove=[veh for veh in vehList if veh.pos > x]
    for veh in toRemove:
        vehList.remove(veh)

def run(downwardEdgeID, distance):
    """execute the TraCI control loop
    """
    global NOAD_ZONE_ENTRY_POS
    # Induction loop IDs for entry detectors 
    loops = ['loop0', 'loop1']
    loopPos = [traci.inductionloop.getPosition(loopID) for loopID in loops]
    loopLanes = [traci.inductionloop.getLaneID(loopID) for loopID in loops]
    # area detectors for densities 
    areaDetectors = ['area0', 'area1']    
    areaPos = [traci.lanearea.getPosition(areaID) for areaID in areaDetectors]
    areaLanes = [traci.lanearea.getLaneID(areaID) for areaID in areaDetectors]
    areaLengths = [traci.lanearea.getLength(areaID) for areaID in areaDetectors]
    assert(areaPos == loopPos)
    assert(areaLanes == loopLanes)
    assert(areaLengths == [NOAD_ZONE_ENTRY_POS - areaPos[0]]*2)
    # Platoons being currently formed (unclosed) on the two lanes ("trailing platoons")
    openPlatoons = [None for _ in loops]
    # Last step detected vehicles (to avoid duplicate additions to platoons) 
    lastStepDetections = [set() for _ in loops]

    # List of CAV/CV platoons in ToC zone. Map: platoon-ID -> platoon object
    platoons = {}
    # List of LVs in ToC zone
    LVsInToCZone = []
    # List of C(A)Vs, which received a TOR 
    pendingToCVehs = []
    
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        # Execute simulation step
        traci.simulationStep()
        t = traci.simulation.getTime()
        step += 1
        print("\n---------------------------------\nstep: %s (time=%s)"%(step, t))
        print("---------------------------------")
        
        removeVehiclesBeyond(NOAD_ZONE_ENTRY_POS, LVsInToCZone)
        removeVehiclesBeyond(NOAD_ZONE_ENTRY_POS, pendingToCVehs)
        
        # Update positions of all detected vehicles
        for plt in platoons.values():
            updatePositions([v for _, v in plt.getVehicles()])
        updatePositions(LVsInToCZone)
        updatePositions(pendingToCVehs)
            
        # Get vehicles that touched the detectors (may include detected of previous step)
        detected = [traci.inductionloop.getLastStepVehicleIDs(loopID) for loopID in loops]
        occupancyLevels = [min(1.0, traci.lanearea.getLastStepOccupancy(areaID)/MAX_OCCUPANCY) for areaID in areaDetectors]
        # iterate over lanes
        for idx in [0,1]:
            loopLane = loopLanes[idx]
            trailingPlatoon = openPlatoons[idx]
            # new vehs on this lane
            ids = [ID for ID in detected[idx] if not ID in lastStepDetections]
            positions = [traci.vehicle.getLanePosition(ID) for ID in ids]
            print("Entered vehicles at lane %s: %s"%(idx, str(ids)))
            print("Occupancy of lane %s: %s"%(idx, occupancyLevels[idx]))
            # Subscribe to automatic state updates for new vehicle
            for vehID in ids:
                subscribeState(vehID)
            
            # List of pairs (id, pos), sorted descendingly in pos
            sortedVehPositions = [(i,p) for (p,i) in reversed(sorted(zip(positions, ids)))]
            # Manage detected vehicles.
            # - Add C(A)Vs to platoons
            # - Add LVs to LVsInToCZone
            for (ID, pos) in sortedVehPositions:
                automationType = getIdentifier(ID, ToC_lead_times.keys())
                # Create Vehicle object
                veh = Vehicle(pos, traci.vehicle.getSpeed(ID), loopLane, \
                              traci.vehicle.getLength(ID), traci.vehicle.getMinGap(ID), \
                              ID, automationType, t)
                if automationType != 'LV.':
                    # Last entered vehicle is automated
                    assert(automationType=="CAVToC." or automationType=="CVToC.")
                    if (loopLane != traci.vehicle.getLaneID(ID)):
                        warn("The detected vehicle's lane differs from entry loop's lane! Please assure that the loop is far enough from the lane end to prevent this situation.")
#                         raise Exception("The detected vehicle's lane differs from entry loop's lane! Please assure that the loop is far enough from the lane end to prevent this situation.")
                    
                    # Add to last platoon or create new platoon
                    if checkPlatoonMembership(trailingPlatoon, veh):
                        trailingPlatoon.appendVeh(veh)
                    else:
                        # If trailing platoon exists, close it
                        if trailingPlatoon is not None:
                            
                            calculateTORPositions(trailingPlatoon, occupancyLevels[idx])
                            trailingPlatoon.close()
                        platoon = Platoon(Platoon._nextPlatoonID, veh)
                        Platoon._nextPlatoonID += 1
                        platoons[platoon.ID] = platoon
                        trailingPlatoon = platoon
                        
                else:
                    # Last entered vehicle is an LV, close current platoon
                    assert(automationType=="LV.")
                    if trailingPlatoon is not None:
                        print("Closing platoon '%s' because of LV detection."%trailingPlatoon.ID)
                        calculateTORPositions(trailingPlatoon, occupancyLevels[idx])
                        trailingPlatoon.close()
                        platoons[trailingPlatoon.ID] = trailingPlatoon 
                    trailingPlatoon = None
                    LVsInToCZone.append(veh)
                    
            if trailingPlatoon is not None:
                # Check if last platoon should be closed
                trailingMember = trailingPlatoon.getTrailer()
                # We need to check the hypothetical TOR positions, to close the platoon if TORs
                # need to be started now (e.g. if there are many vehicles in the platoon filling
                # the complete area)
                calculateTORPositions(trailingPlatoon, occupancyLevels[idx])
                closePlatoon = False
                msg = "Closing platoon '%s' because "%trailingPlatoon.ID
                if trailingMember.lane != loopLanes[idx]:
                    msg += "it deviated from the detector lane (%s->%s)."%(loopLanes[idx], trailingMember.lane)
                    closePlatoon = True
                elif trailingMember.pos > loopPos[idx] + PLATOON_CLOSING_DIST:
                    msg += "its distance to the detector exceeded PLATOON_CLOSING_DIST(=%s)"%PLATOON_CLOSING_DIST
                    closePlatoon = True
                elif t - trailingMember.detectionTime > PLATOON_CLOSING_TIME:
                    msg += "the time since the last join exceeded PLATOON_CLOSING_TIME(=%s)"%PLATOON_CLOSING_TIME
                    closePlatoon = True
                else:                        
                    # minimal distance of a platoon member to its TOR point. 
                    # If this is negative, a TOR should be issued immediately
                    TORDist = min([veh.xTOR - veh.pos for _, veh in trailingPlatoon.getVehicles()])
                    if TORDist <= 0:
                        msg += "a vehicle passed its xTOR by %s"%(-TORDist)
                        closePlatoon = True
                if closePlatoon:
                    # Close it, TOR positions are already calculated
                    print(msg)
                    trailingPlatoon.close()
                    platoons[trailingPlatoon.ID] = trailingPlatoon 
                    trailingPlatoon=None
            # Store current trailing platoon in openPlatoons
            openPlatoons[idx] = trailingPlatoon
        
        # Update vehicle detections in last step.
        lastStepDetections = set()
        for vehs in detected:
            lastStepDetections.update(vehs)  
        
        # Iterate over platoons and issue TORs, where appropriate
        emptyPlatoons=[]
        for plt in platoons.values():
            if plt.isClosed:
                # Update TOR positions depending on current speed
                updateTORPositions(plt)
                # Pop vehicles, for which TOR was applied and put into pendingToCVehs 
                removed = issueTORs(plt)
                pendingToCVehs.extend(removed)
                if len(plt.vehs) == 0:
                    # All vehicles have a TOR, now
                    emptyPlatoons.append(plt.ID)
        # remove empty platoons
        if emptyPlatoons:
            print("Removing empty platoons: %s"%str(emptyPlatoons))
        for pltID in emptyPlatoons:
            platoons.pop(pltID)
        
        
#         print('Nr. of vehicles from the start: ',len(numberOfVehicles))
#         print('Nr. of detected vehicles at detector: ',len(detectedVehicles))
        print('Nr. of platoons: ',len(platoons))
                

def requestToC(vehID, timeUntilMRM):
    traci.vehicle.setParameter(vehID, "device.toc.requestToC", str(timeUntilMRM))


def printToCParams(vehID, only_dynamic=False):
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


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-c", dest="sumocfg", help="sumo config file")
    optParser.add_option("--suffix", dest="suffix", help="suffix for output filenames")
    optParser.add_option("--seed", dest="seed", help="seed value")
    optParser.add_option("--additionals", dest="additionals", help="filename list for additional files to be loaded")
    optParser.add_option("--routes", dest="routes", help="filename list of route files to be loaded")
    optParser.add_option("--gui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    optParser.add_option("-v", "--verbose", action="store_true", dest="verbose",
                         default=False, help="tell me what you are doing")
    optParser.add_option("-d", "--debug", action="store_true", dest="debug",
                         default=False, help="debug messages")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()
    downwardEdgeID = None
    distance = None
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.gui:
        sumoBinary = checkBinary('sumo-gui')
    else:
        sumoBinary = checkBinary('sumo-gui')

#     traci.start([sumoBinary, "-c", options.sumocfg, "--seed", options.seed, 
#     "--summary", "outputSummary%s.xml"%options.suffix, "-a", options.additionals, "-r", options.routes,
#     "--lanechange-output", "outputLaneChanges%s.xml"%options.suffix,
#     "--queue-output", "outputQueue%s"%options.suffix])
        
    sumo_args = ["-c", "uc5.sumo.cfg", "-r", "routes_trafficMix_0_trafficDemand_1_driverBehaviour_OS_seed_0.xml", "-a", "additionalsOutput_trafficMix_0_trafficDemand_1_driverBehaviour_OS_seed_0.xml, vTypesCAVToC_OS.add.xml, vTypesCVToC_OS.add.xml, vTypesLV_OS.add.xml"]
    traci.start([sumoBinary] + sumo_args)
    
    downwardEdgeID = "e0"
    distance = 2300. 
    #~ upwardEdgeID = "e0"
    #~ upwardDist = 3500.0

    run(downwardEdgeID, distance) #, upwardEdgeID, upwardDist)
    
    traci.close()
    sys.stdout.flush()
    
    
