from plan.base_planer import BasePlanner
from util import time_const, handle_exception,compute_distance2D, init_data_file
from plan.carFollowingModel import Path_CACC, Path_ACC,  Adaptive_CACCController
import carla
import time

class CACCPlanner(BasePlanner):
    def __init__(self, world, map,start_point,end_point, vehicle,vehicle_info, config, global_route_planner,controller, sensor_manager, commuic_agent):
        super().__init__(world, map,start_point,end_point, vehicle,vehicle_info, config, global_route_planner,controller, sensor_manager, commuic_agent)
        self.state = "ACC" if self.config["topology"]["LV"] == -1 else "CACC"
        if self.state == "ACC":
            pass
            # self.vehicle.show_debug_telemetry(True)
        self.plt_info = {}
        self.fp , self.writer = init_data_file("../data/ind", self.id+".csv")
        self.writer.writerow(
            [
                "time_step",
                "time",
                "leader_v",
                "leader_a",
                "distance_front",
                "front_v",
                "front_a",
                "leader_delay",
                "front_delay"
            ]
        )
        self.cacc_model = Path_ACC(
        ) if self.config["topology"]["LV"] == -1 else Path_CACC()
        # ) if self.config["topology"]["LV"] == -1 else IDM()
        # ) if self.config["topology"]["LV"] == -1 else Adaptive_CACCController()


    @time_const(fps=30) 
    def run_step(self, obs):
        try:
            self.ego_state_update()
            self.obstacle_update(obs)
            self.check_waypoints()
            if len(self.trajectories) < 1:
                return
            self.check_trajectories()
            if len(self.trajectories) < 1:
                return
            self.check_traffic_light()
            self.side_safety_check()
            # self.debug()
            self.step += 1
            self.change_back_step += 1
            self.send_self_info()
            self.get_plt_info()
            if self.state != "CACC":
                self.check_collison_and_change_lanes()
                self.change_speed_profile()
            self.car_following()
            self.run_control()
        except Exception as e:
            handle_exception(e)
            
    def change_speed_profile(self, time_interval=350, low_speed=15, high_speed=20):
        # Calculate the current interval index based on the step
        current_interval = (self.step // time_interval) % 2  # This will alternate between 0 and 1

        # Determine the max speed based on the current interval
        if 0 < self.step:
            self.max_speed = low_speed if current_interval == 0 else high_speed

    def get_plt_info(self):
        vehicle_types = [
            key for key, value in self.config["topology"].items() if value != -1 and key not in ["index", "len"]]
        vehicle_info = {}
        self_index = self.config["topology"]["index"]
        if self_index == 1:
            vehicle_info["LV"] = self.communi_agent.rec_obj("LV")
            vehicle_info["FV"] = vehicle_info["LV"]
            vehicle_info["RV"] = self.communi_agent.rec_obj("RV")
            self.plt_info =  vehicle_info
            return

        for vehicle_type in vehicle_types:
            vehicle_info[vehicle_type] = self.communi_agent.rec_obj(vehicle_type)
        self.plt_info =  vehicle_info

    def send_self_info(self):
        self.communi_agent.send_obj({
            "id": self.id,
            "speed": self.speed,
            "target_speed": self.target_speed,
            "acc": self.acc,
            "xy": [self.location.x, self.location.y],
            "yaw": self.transform.rotation.yaw,
            "state": self.state,
            "step": self.step,
            "time": time.time()
        })
    def car_following(self):
        if self.state not in ["CACC"]:
            if self.use_car_following:
                leading_vehicle = self.sensor_manager.radar_res["front"]
                leading_obs = self.sensor_manager.obstacle
                if leading_vehicle:
                    front_v = leading_vehicle[1] + self.speed
                    distance_s = leading_vehicle[0]
                    v = self.car_following_model.calc_speed(
                        front_v, distance_s, self.speed)
                    self.target_speed = max(0, self.speed + v)
                elif leading_obs:
                    distance_s = leading_obs.distance
                    front_v = leading_obs.velocity
                    v = self.car_following_model.calc_speed(
                        front_v, distance_s, self.speed)
                    self.target_speed = max(0, self.speed + v)
                else:
                    pass
                self.target_waypoint = carla.Transform(
                    carla.Location(x=self.trajectories[0][0], y=self.trajectories[0][1]))
                return 0 # IDM
        plt_info_lv = self.plt_info.get("LV")
        plt_info_fv = self.plt_info.get("FV")
        plt_info_rv = self.plt_info.get("RV")
        leader_v = plt_info_lv.get("speed") if plt_info_lv else None
        leader_tv = plt_info_lv.get("target_speed") if plt_info_lv else None
        leader_t = plt_info_lv.get("time") if plt_info_lv else None
        leader_a = plt_info_lv.get("acc") if plt_info_lv else None
        self.leader_xy = plt_info_lv.get("xy") if plt_info_lv else None
        front_v = plt_info_fv.get("speed") if plt_info_fv else None
        front_t = plt_info_fv.get("time") if plt_info_fv else None
        front_a = plt_info_fv.get("acc") if plt_info_fv else None
        self.front_xy = plt_info_fv.get("xy") if plt_info_fv else None
        back_xy = plt_info_rv.get("xy") if plt_info_rv else None
        ego_xy = [self.location.x, self.location.y]
        leader_dis = None
        front_dis = None
        back_dis = None
        leader_delay = 0.05
        front_delay = 0.05
        if self.front_xy:
            front_dis = compute_distance2D(ego_xy, self.front_xy)-self.vehicle_info["length"] - self.vehicle_info["trailer_length"]*2
        if self.leader_xy:
            self.leader_dis = compute_distance2D(ego_xy, self.leader_xy)-self.vehicle_info["length"] - self.vehicle_info["trailer_length"]*2
        if self.sensor_manager.radar_res["front"]:
            front_dis = self.sensor_manager.radar_res["front"][0]
            front_v = self.sensor_manager.radar_res["front"][1]+ self.speed
            # print(front_dis, self.id)
        if back_xy:
            back_dis = compute_distance2D(ego_xy, back_xy) - self.vehicle_info["trailer_length"]*2 - self.vehicle_info["length"]
        elif self.sensor_manager.radar_res["rear"]:
            back_dis = self.sensor_manager.radar_res["rear"][0]-2.9320433139801025
            # back_dis = self.sensor_manager.radar_res["rear"][0]
        ego_t = time.time()
        if leader_t:
            leader_delay = ego_t - leader_t
        if front_t:
            front_delay = ego_t - front_t
        if self.state == "ACC":
            if back_dis:
                self.target_speed =  self.cacc_model.calc_speed(
                        self.max_speed, self.speed, back_dis)
                self.target_waypoint = carla.Transform(
                    carla.Location(x=self.trajectories[0][0], y=self.trajectories[0][1]))
        if self.state == "CACC":
            self.writer.writerow([
                self.step,
                ego_t,
                leader_v,
                leader_a,
                front_dis,
                front_v,
                front_a,
                leader_delay,
                front_delay,
            ])
            if front_dis and leader_v and front_v and front_a and leader_a:
                    self.target_speed = self.cacc_model.calc_speed(leader_v,
                    leader_a,
                    front_dis,
                    front_v,
                    front_a,
                    self.speed,
                    self.acc,
                    leader_delay,
                    front_delay,
                    leader_tv
                    )
                    self.target_waypoint = carla.Transform(
                                    carla.Location(x=self.front_xy[0], y=self.front_xy[1]))
            else:
                self.target_waypoint= carla.Transform(
                carla.Location(x=self.trajectories[0][0], y=self.trajectories[0][1]))
 

        