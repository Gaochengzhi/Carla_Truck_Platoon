from plan.base_planer import BasePlanner
from util import time_const, handle_exception,compute_distance2D
from plan.carFollowingModel import PLF_Controller, Path_ACC, BDL_Controller, Path_CACC
import carla

class CACCPlanner(BasePlanner):
    def __init__(self, world, map,start_point,end_point, vehicle,vehicle_info, config, global_route_planner,controller, sensor_manager, commuic_agent):
        super().__init__(world, map,start_point,end_point, vehicle,vehicle_info, config, global_route_planner,controller, sensor_manager, commuic_agent)
        self.state = "ACC" if self.config["topology"]["LV"] == -1 else "CACC"
        self.plt_info = {}
        self.cacc_model = Path_ACC(
        ) if self.config["topology"]["LV"] == -1 else PLF_Controller()

    @time_const(fps=30) 
    def run_step(self, obs):
        try:
            self.ego_state_update()
            self.send_self_info()
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
            if self.state != "CACC":
                self.check_collison_and_change_lanes()
            self.car_following()
            self.run_control()
        except Exception as e:
            handle_exception(e)

    def get_plt_info(self):
        vehicle_types = [
            key for key, value in self.config["topology"].items() if value != -1]
        vehicle_info = {}
        for vehicle_type in vehicle_types:
            vehicle_info[vehicle_type] = self.communi_agent.rec_obj(vehicle_type)
        self.plt_info =  vehicle_info

    def send_self_info(self):
        self.communi_agent.send_obj({
            "id": self.id,
            "speed": self.speed,
            "acc": self.acc,
            "xy": [self.location.x, self.location.y],
            "yaw": self.transform.rotation.yaw,
            "state": self.state,
            "step": self.step
        })
    def car_following(self):
        if self.state not in ["CACC"]:
            if self.use_car_following:
                leading_vehicle = self.sensor_manager.radar_res["front"]
                leading_obs = self.sensor_manager.obstacle
                if leading_vehicle:
                    front_v = leading_vehicle[1] + self.speed
                    distance_s = leading_vehicle[0]
                    v = self.car_following_model.calc_acc(
                        front_v, distance_s, self.speed)
                    self.target_speed = max(0, self.speed + v)
                elif leading_obs:
                    distance_s = leading_obs.distance
                    front_v = leading_obs.velocity
                    v = self.car_following_model.calc_acc(
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
        leading_v = plt_info_lv.get("speed") if plt_info_lv else None
        leading_a = plt_info_lv.get("acc") if plt_info_lv else None
        front_v = plt_info_fv.get("speed") if plt_info_fv else None
        front_a = plt_info_fv.get("acc") if plt_info_fv else None
        self.front_xy = plt_info_fv.get("xy") if plt_info_fv else None
        back_xy = plt_info_rv.get("xy") if plt_info_rv else None
        ego_xy = [self.location.x, self.location.y]
        front_dis = None
        back_dis = None
        if self.front_xy:
            front_dis = compute_distance2D(ego_xy, self.front_xy)-self.vehicle_info["length"] - self.vehicle_info["trailer_length"]*2
        elif self.sensor_manager.radar_res["front"]:
            front_dis = self.sensor_manager.radar_res["front"][0]
        if back_xy:
            back_dis = compute_distance2D(ego_xy, back_xy) - self.vehicle_info["trailer_length"]*2 - self.vehicle_info["length"]
        elif self.sensor_manager.radar_res["rear"]:
            back_dis = self.sensor_manager.radar_res["rear"][0]-5.4
        
        if self.state == "ACC":
            self.target_speed =  self.cacc_model.calc_acc(
                    13, self.speed, back_dis)
            self.target_waypoint = carla.Transform(
                carla.Location(x=self.trajectories[0][0], y=self.trajectories[0][1]))
            return 1
        if self.state == "CACC":
            if front_dis and leading_v and front_v and front_a and leading_a:
                    self.target_speed = self.cacc_model.calc_speed(front_dis, front_v, front_a, leading_v, leading_a, self.speed, self.acc)
                    self.target_waypoint = carla.Transform(
                                    carla.Location(x=self.front_xy[0], y=self.front_xy[1]))
            else:
                self.target_waypoint= carla.Transform(
                carla.Location(x=self.trajectories[0][0], y=self.trajectories[0][1]))
 

        