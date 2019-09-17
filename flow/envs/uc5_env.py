from flow.envs import Env
import numpy as np

ToC_lead_times = {"CAVToC.":10.0, "CVToC.":0.0}

# define the environment class myEnv, and inherit properties from the base environment class Env
class uc5_env(Env):
    def __init__(self, env_params, sim_params, scenario, simulator, stepListener):
        super().__init__(env_params, sim_params, scenario, simulator,stepListener)
#         self.segments = add_env_params.get("controlled_segments", default)
# 
#         # number of segments for each edge
#         self.num_segments = [segment[1] for segment in self.segments]
# 
#         # whether an edge is controlled
#         self.is_controlled = [segment[2] for segment in self.segments]
# 
#         self.num_controlled_segments = [
#             segment[1] for segment in self.segments if segment[2]
#         ]
# 
#         # sum of segments
#         self.total_segments = int(
#             np.sum([segment[1] for segment in self.segments]))
#         # sum of controlled segments
#         segment_list = [segment[1] for segment in self.segments if segment[2]]
#         self.total_controlled_segments = int(np.sum(segment_list))
# 
#         # list of controlled edges for comparison
#         self.controlled_edges = [
#             segment[0] for segment in self.segments if segment[2]
#         ]
# 
# #         additional_params = env_params.additional_params
# 
#         # for convenience, construct the relevant positions defining
#         # segments within edges
#         # self.slices is a dictionary mapping
#         # edge (str) -> segment start location (list of int)
#         self.slices = {}
#         for edge, num_segments, _ in self.segments:
#             edge_length = self.k.scenario.edge_length(edge)
#             self.slices[edge] = np.linspace(0, edge_length, num_segments + 1)
# 
#         # get info for observed segments
#         self.obs_segments = additional_params.get("observed_segments", [])
# 
#         # number of segments for each edge
#         self.num_obs_segments = [segment[1] for segment in self.obs_segments]
# 
#         # for convenience, construct the relevant positions defining
#         # segments within edges
#         # self.slices is a dictionary mapping
#         # edge (str) -> segment start location (list of int)
#         self.obs_slices = {}
#         for edge, num_segments in self.obs_segments:
#             edge_length = self.k.scenario.edge_length(edge)
#             self.obs_slices[edge] = np.linspace(0, edge_length,
#                                                 num_segments + 1)
# 
#         # self.symmetric is True if all lanes in a segment
#         # have same action, else False
#         self.symmetric = additional_params.get("symmetric")
# 
#         # action index tells us, given an edge and a lane,the offset into
#         # rl_actions that we should take.
#         self.action_index = [0]
#         for i, (edge, segment, controlled) in enumerate(self.segments[:-1]):
#             if self.symmetric:
#                 self.action_index += [
#                     self.action_index[i] + segment * controlled
#                 ]
#             else:
#                 num_lanes = self.k.scenario.num_lanes(edge)
#                 self.action_index += [
#                     self.action_index[i] + segment * controlled * num_lanes
#                 ]
# 
#         self.action_index = {}
#         action_list = [0]
#         index = 0
#         for (edge, num_segments, controlled) in self.segments:
#             if controlled:
#                 if self.symmetric:
#                     self.action_index[edge] = [action_list[index]]
#                     action_list += [action_list[index] + controlled]
#                 else:
#                     num_lanes = self.k.scenario.num_lanes(edge)
#                     self.action_index[edge] = [action_list[index]]
#                     action_list += [
#                         action_list[index] +
#                         num_segments * controlled * num_lanes
#                     ]
#                 index += 1
    
    def getIdentifier(self,fullID, identifierList):
        for start_str in identifierList:
            if fullID.startswith(start_str):
                return start_str
    @property
    def action_space(self):
        # from flow guys
        if self.symmetric:
            action_size = self.total_controlled_segments
        else:
            action_size = 0.0
            for segment in self.segments:  # iterate over segments
                if segment[2]:  # if controlled
                    num_lanes = self.k.scenario.num_lanes(segment[0])
                    action_size += num_lanes * segment[1]
        return Box(low=0, high=1, shape=(int(action_size), ), dtype=np.int32)
    @property
    def observation_space(self):
#         # from flow guys
#         num_obs = 0
#         # density and velocity for rl and non-rl vehicles per segment
#         # Last element is the outflow
#         for segment in self.obs_segments:
#             num_obs += 4 * segment[1] * self.k.scenario.num_lanes(segment[0])
#         num_obs += 1
        return Box(low=0.0, high=1.0, shape=(num_obs, ), dtype=np.int32)

    def _apply_rl_actions(self, rl_actions):
        departedToCVehs = [vehID for vehID in self.k.vehicle.get_departed_ids() if self.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
        for rl_id in departedToCVehs:
            edge = self.k.vehicle.get_edge(rl_id)
            lane = self.k.vehicle.get_lane(rl_id)
            if edge:
                # If in outer lanes, on a controlled edge, in a controlled lane
                if edge[0] != ':' and edge in self.controlled_edges:
                    pos = self.k.vehicle.get_position(rl_id)
 
                    if not self.symmetric:
                        num_lanes = self.k.scenario.num_lanes(edge)
                        # find what segment we fall into
                        bucket = np.searchsorted(self.slices[edge], pos) - 1
                        action = rl_actions[int(lane) + bucket * num_lanes +
                                            self.action_index[edge]]
                    else:
                        # find what segment we fall into
                        bucket = np.searchsorted(self.slices[edge], pos) - 1
                        action = rl_actions[bucket + self.action_index[edge]]
#                     max_speed_curr = self.k.vehicle.get_max_speed(rl_id)
#                     next_max = np.clip(max_speed_curr + action, 0.01, 23.0)
#                     self.k.vehicle.set_max_speed(rl_id, next_max)
                    if action == 1:
                        ToCVehicleType = self.getIdentifier(rl_id, ToC_lead_times.keys())
                        timeUntilMRM=ToC_lead_times[ToCVehicleType]
                        self.k.vehicle.setParameter(rl_id, "device.toc.requestToC", str(timeUntilMRM))

###################################################################################################        
#         arrivedVehs = [vehID for vehID in self.k.simulation.getArrivedIDList() if self.k.scenario.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
#         self.k.scenario.downwardToCRequested.difference_update(arrivedVehs)
#         self.k.scenario.downwardToCPending.difference_update(arrivedVehs)
#         departedToCVehs = [vehID for vehID in self.k.simulation.getDepartedIDList() if self.k.scenario.getIdentifier(vehID, ToC_lead_times.keys()) is not None]
#         for vehID in departedToCVehs:
#             # set ToC vehicle class to custom1
#             self.k.vehicle.setVehicleClass(vehID, "custom1")
# 
#         vehSet=self.k.scenario.downwardToCPending.update(departedToCVehs)
# 
#         for vehID in vehSet:
#             ToCVehicleType = self.getIdentifier(vehID, ToC_lead_times.keys())
#             timeUntilMRM=ToC_lead_times[ToCVehicleType]
#             if ToCVehicleType is not None:
#                 # Request a ToC for the vehicle
#                 if rl_actions[vehID] == 1: # this WILL CRASH - nr of actions does not match the number of current CAVs
#                     self.k.vehicle.setParameter(vehID, "device.toc.requestToC", str(timeUntilMRM))
#                 
# #             newTORs.append(vehID)
# #             handdledSet.add(vehID)

#         self.scenario.downwardToCPending.difference_update(self.scenario.downwardToCRequested)
#         # keep book on performed ToCs and trigger best lanes update by resetting the route
#         downwardToCPerformed = set()
#         for vehID in self.k.scenario.downwardToCRequested:
#             if self.k.vehicle.getVehicleClass(vehID) == "passenger":
#                 if debug:
#                     print("Downward transition completed for vehicle '%s'" % vehID)
#                 downwardToCPerformed.add(vehID)
#                 self.k.vehicle.updateBestLanes(vehID)
#         self.k.scenario.downwardToCRequested.difference_update(downwardToCPerformed)
###################################################################################################
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