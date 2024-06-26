import time
import functools
import numpy as np
import math
import random
import sys
import carla
from util import compute_distance, log_time_cost, compute_3D21d, compute_distance2D, interpolate_points, time_const, handle_exception
from cythoncode.cutil import is_within_distance_obs, compute_magnitude_angle
from view.debug_manager import draw_waypoints, draw_list
from scipy.interpolate import splprep, splev
from plan.carFollowingModel import IDM, PATH_CACC, Path_ACC, BDL_Controller, PATH_CACC
from perception.perception_basline import FakePerception
import logging
from pyinstrument import Profiler


class BasePlanner():
    def __init__(self, world, map,start_point,end_point, vehicle,vehicle_info, config, global_route_planner,controller, sensor_manager, commuic_agent):
        self.start_point = start_point
        self.end_point = end_point
        self.global_route_planner = global_route_planner
        self.global_waypoints = [x[0] for x in self.global_route_planner.trace_route(
        self.start_point.location, self.end_point.location)]
        self.world = world
        self.map = map
        self.vehicle_info = vehicle_info
        self.config = config
        self.state = "ACC" if self.config["topology"]["LV"] == -1 else "CACC"
        self.vehicle = vehicle
        self.id = vehicle.attributes["role_name"]
        self.obs_list = []
        self.waypoint_buffer = []
        self.trajectories = []
        self.target_offset = 0
        self.current_offset = 0
        self.speed = 0
        self.acc = 0
        self.plt_info = {}
        self.transform = None
        self.location = None
        self.controller = controller
        self.sensor_manager = sensor_manager
        self.communi_agent = commuic_agent
        self.perception = FakePerception(vehicle, config)
        self.car_following_model = IDM()
        self.cacc_model = Path_ACC(
        ) if self.config["topology"]["LV"] == -1 else PATH_CACC()
        self.use_car_following = False
        self.max_speed = self.config.get("max_speed", 30)
        self.target_speed = self.max_speed*0.8
        self.detect_range = 2.6
        self.front_xy = []
        self.step = 0
        self.change_back_step = 0
        self.wait_step = 4
        self.vehicle.show_debug_telemetry(True)
        self.check_update_waypoints()
        self.left_side, self.right_side = self.perception.get_road_edge(
            self.waypoint_buffer[0])
        # self.profiler = Profiler(interval=0.001)

    def adjust_offset(self, direction):
        self.target_offset += direction
        self.update_trajectories(self.target_offset)
        if self.check_collision():
            self.target_offset -= direction
            self.update_trajectories(self.target_offset)

    @time_const(fps=30)
    # @log_time_cost(name="ego_planner")
    def run_step(self, obs):
        try:
            # self.profiler.start()
            self.ego_state_update()
            self.obstacle_update(obs)
            self.check_waypoints()
            if len(self.trajectories) < 1:
                return
            self.check_trajectories()
            self.plt_info = self.get_plt_info()
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
            front_dis = 0
            back_dis = 0
            if self.front_xy:
                front_dis = compute_distance2D(ego_xy, self.front_xy)-self.vehicle_info["length"] - self.vehicle_info["trailer_length"]*2
            elif self.sensor_manager.radar_res["front"]:
                front_dis = self.sensor_manager.radar_res["front"][0]

            else:
                pass

            if back_xy:
                back_dis = compute_distance2D(ego_xy, back_xy) - self.vehicle_info["trailer_length"]*2 - self.vehicle_info["length"]
                # print(back_dis, "dis", self.id)

            if self.sensor_manager.radar_res["rear"]:
                back_dis = self.sensor_manager.radar_res["rear"][0]-5.4
                # print(back_dis, "radarbdis", self.vehicle.attributes["role_name"])

            # else:
            #     # back_dis = 1
            #     pass

            if self.state == "CACC":
                if front_dis and leading_v and front_v and front_a and leading_a:
                    self.target_speed = self.cacc_model.calc_speed(front_dis, front_v, front_a, leading_v, leading_a, self.speed, self.acc)

                # if front_dis and back_dis and leading_v:
                #     self.target_speed = self.cacc_model.calc_speed(
                #        front_dis, back_dis, self.speed,leading_v)
                # if front_dis and front_v and front_a:
                #     self.target_speed = self.speed +  self.cacc_model.calc_speed(
                #         front_dis,front_v, self.speed, front_a)
                #     print(self.target_speed, self.vehicle.attributes["role_name"])
                else:
                    pass
                if self.front_xy:
                    target_waypoint = carla.Transform(
                        carla.Location(x=self.front_xy[0], y=self.front_xy[1]))
                    control = self.controller.run_step(
                        self.target_speed, target_waypoint)
                    self.vehicle.apply_control(control)
                self.step += 1
                return
            else:
                self.check_collison_and_change_lanes()
                # print(self.vehicle.attributes["role_name"], self.target_speed)
            if len(self.trajectories) < 1:
                return
            # DEBUG
            draw_waypoints(self.world, self.waypoint_buffer,
                           z=2, life_time=0.05, size=0.05)
            draw_list(self.world, self.trajectories, size=0.05,
                      color=carla.Color(0, 250, 123), life_time=0.05)

            if self.use_car_following:
                # print("use_car_following")
                leading_vehicle = self.sensor_manager.radar_res["front"]
                leading_obs = self.sensor_manager.obstacle
                if leading_vehicle:
                    front_v = leading_vehicle[1] + self.speed
                    distance_s = leading_vehicle[0]
                    a = self.car_following_model.calc_speed(
                        front_v, distance_s, self.speed)
                    self.target_speed = max(0, self.speed + a)
                elif leading_obs:
                    distance_s = leading_obs.distance
                    front_v = leading_obs.velocity
                    a = self.car_following_model.calc_speed(
                        front_v, distance_s, self.speed)
                    self.target_speed = max(0, self.speed + a)
                else:
                    # self.target_speed = 0
                    pass



            self.check_traffic_light()
            self.update_current_offset()
            angle = self.get_relative_waypoint_angle()
            if self.check_radar(angle):
                self.use_car_following = True
                self.target_offset = self.current_offset
                self.update_trajectories(self.target_offset)
            else:
                self.target_speed = min(
                    self.max_speed, self.target_speed*1.5)
            # CONTROLLER
            target_waypoint = carla.Transform(
                carla.Location(x=self.trajectories[0][0], y=self.trajectories[0][1]))
            
            if self.state == "ACC":
                a = self.cacc_model.calc_speed(
                    33, self.speed, back_dis)
                self.target_speed = a
                # print(back_dis)
            # print(self.target_speed, self.vehicle.attributes["role_name"])
            # if self.step >= 800:
            #     self.target_speed = 15
            control = self.controller.run_step(
                self.target_speed, target_waypoint)

            self.vehicle.apply_control(control)
            self.step += 1
            self.change_back_step += 1
            # self.profiler.stop()
            # self.profiler.print(show_all=True)

        except Exception as e:
            handle_exception(e)


    def check_collison_and_change_lanes(self):
        if self.step % self.wait_step == 0:
            if self.check_collision():
                self.adjust_trajectories()
                self.wait_step += 1
            else:
                self.use_car_following = False
                self.wait_step = 4
                self.target_speed = min(
                    self.max_speed, self.speed*1.1)
                if self.change_back_step > 180:
                    if self.target_offset > 0.3 and self.check_radar(-30):
                        self.adjust_offset(-0.5)
                    elif self.target_offset < -0.3 and self.check_radar(30):
                        self.adjust_offset(0.5)

    def check_radar(self, offset=0):
        """
        Check if there is an obstacle in the specified direction.

        Negative offset checks left, positive offset checks right, 0 checks front
        return True if there is an obstacle
        """
        direction = "left" if offset < -1 else "right" if offset > 1 else "front"
        return self.sensor_manager.radar_res.get(direction)

    def check_trajectories(self):
        if compute_distance2D(self.trajectories[0], [self.location.x, self.location.y]) < 6:
            self.trajectories.pop(0)
            self.update_trajectories(self.target_offset)

    def check_waypoints(self):
        distance_to_next_waypoint = compute_distance(
            self.waypoint_buffer[0].transform.location, self.location)
        if distance_to_next_waypoint < 3+abs(int(self.target_offset)):
            self.waypoint_buffer.pop(0)
            self.global_waypoints.pop(0)
            if len(self.global_waypoints) < 1:
                logging.info("finish the route!")
                while True:
                    self.vehicle.apply_control(
                        carla.VehicleControl(throttle=0.0, brake=1))

            self.update_waypoint_buffer()
            self.left_side, self.right_side = self.perception.get_road_edge(
                self.waypoint_buffer[0])
            self.check_road_edge()

    def check_road_edge(self):
        if self.target_offset > self.right_side:
            self.target_offset = self.right_side
        if self.target_offset < -self.left_side:
            self.target_offset = -self.left_side

    def adjust_trajectories(self):
        traj_adjust_options = np.concatenate(
            (np.arange(self.target_offset, -self.left_side - 1, -1), 
            np.arange(self.target_offset+1, self.right_side, 1)))
        collision_info = []

        for offset in traj_adjust_options:
            self.update_trajectories(offset)
            collision_result = self.check_collision()

            if collision_result:
                highest_velocity = max(collision_result, key=lambda x: x[1])[1]
                nearest_index = min(collision_result, key=lambda x: x[2])[2]
                collision_info.append((offset, highest_velocity, nearest_index))
            else:
                self.change_back_step = 0
                self.target_offset = offset - 3 if offset > 0 else offset + 3 if abs(offset-self.current_offset) > 4 else offset
                self.use_car_following = True
                return False if abs(offset-self.current_offset) > 4 else True

        if collision_info:
            chosen_offset, chosen_velocity, _ = max(collision_info, key=lambda x: (x[1], -x[2]))
            self.use_car_following = True
            velocity_threshold = 2  
            if chosen_velocity >= self.speed + velocity_threshold:
                self.target_offset = chosen_offset - 3 if offset > 0 else chosen_offset + 3 if abs(chosen_offset-self.current_offset) > 4 else chosen_offset

    def check_update_waypoints(self):
        if len(self.waypoint_buffer) < 1:
            self.update_waypoint_buffer()
        if len(self.trajectories) < 1:
            self.update_waypoint_buffer()
            self.update_trajectories(self.target_offset)

    # @log_time_cost(name="update_trajectories")
    def update_trajectories(self, offset=0):
        xy_list = np.array([[waypoint.transform.location.x, waypoint.transform.location.y]
                            for waypoint in self.waypoint_buffer])
        lenxy = len(xy_list)

        if lenxy > 6:
            tck, _ = splprep(xy_list.T, s=min(lenxy, int(self.speed*2)), k=5)
        elif lenxy > 1:
            tck, _ = splprep(xy_list.T, s=8, k=lenxy-1)
        else:
            self.trajectories = xy_list.tolist()
            return

        u_new = np.linspace(0, 1, lenxy * 2)
        x_fine, y_fine = splev(u_new, tck)
        interpolated_waypoints = np.column_stack([x_fine, y_fine])

        if abs(offset) > 0.1:
            directions = np.diff(interpolated_waypoints, axis=0)
            normals = np.array([-directions[:, 1], directions[:, 0]]).T
            normals /= np.sqrt((normals ** 2).sum(axis=1)).reshape(-1, 1)
            interpolated_waypoints += np.vstack([[normals[0]], normals]) * offset
            interpolated_waypoints = interpolated_waypoints[min(len(interpolated_waypoints)-1, int(self.speed)):]

        self.trajectories = interpolated_waypoints.tolist()

    def check_collision(self):
        """
        collision_info: [obstacle, velocity, index(distance)]
        """
        start_location = [self.location.x, self.location.y]
        end_location = self.trajectories[0]
        interpolated_points = interpolate_points(start_location, end_location, 2)[1:]
        self.trajectories = interpolated_points + self.trajectories
        draw_list(self.world, interpolated_points, size=0.1, color=carla.Color(110, 25, 144), life_time=0.15)
        collision_info = [(ob, ob[2], index) for ob in self.obs_list for index, point in enumerate(self.trajectories) 
                          if compute_distance2D((point[0], point[1]), (ob[0], ob[1])) < self.detect_range]

        return collision_info if collision_info else False

    def update_waypoint_buffer(self):
        num_push = min(int(self.speed/1.5)+3, len(self.global_waypoints), 30)
        self.waypoint_buffer = self.global_waypoints[:num_push]
        if num_push < 3:
            return
        curvature = sum(self.calculate_curvature(i) for i in range(1, num_push - 1))

        if curvature > 0.0001:
            self.target_speed = max(10, self.max_speed * math.exp(-6*curvature))
        else:
            self.target_speed = self.max_speed

    def calculate_curvature(self, i):
        point1 = self.waypoint_buffer[i-1].transform.location
        point2 = self.waypoint_buffer[i].transform.location
        point3 = self.waypoint_buffer[i+1].transform.location
        vector1 = self.normalize_vector(point2.x - point1.x, point2.y - point1.y)
        vector2 = self.normalize_vector(point3.x - point2.x, point3.y - point2.y)
        dot_product = vector1[0]*vector2[0] + vector1[1]*vector2[1]
        return 1 - dot_product

    @staticmethod
    def normalize_vector(dx, dy):
        mag = (dx**2 + dy**2)**0.5
        return dx / mag, dy / mag if mag > 0 else (0, 0)

    def obstacle_update(self, obs):
        if not obs:
            return
        ego_location = np.array([self.transform.location.x, self.transform.location.y], dtype=np.float32)
        ego_yaw = self.transform.rotation.yaw
        self.obs_list = [[obs_info["location"].x, obs_info["location"].y, obs_info["velocity"], obs_info["yaw"]] 
                         for obs_info in obs if obs_info["id"] != self.id and 
                         is_within_distance_obs(ego_location, np.array([obs_info["location"].x, obs_info["location"].y], dtype=np.float32), 
                                                50, self.speed, obs_info["velocity"], ego_yaw, [-65, 65])]
        self.communi_agent.send_obj({
            "id": self.id,
            "speed": self.speed,
            "acc": self.acc,
            "xy": [self.location.x, self.location.y],
            "yaw": self.transform.rotation.yaw,
            "state": self.state,
            "step": self.step
        })

    def compute_brake(self, distance):
        brake = math.exp(-distance/10)
        return brake**0.4

    def ego_state_update(self):
        self.location, self.transform, self.speed, self.acc = self.perception.get_ego_vehicle_info()

    def get_relative_waypoint_angle(self):
        if not self.waypoint_buffer:
            return 0
        waypoint = self.waypoint_buffer[0]
        waypoint_yaw_rad = math.radians(waypoint.transform.rotation.yaw)
        ego_yaw_rad = math.radians(self.transform.rotation.yaw)
        relative_angle_rad = (ego_yaw_rad - waypoint_yaw_rad + math.pi) % (2 * math.pi) - math.pi
        return math.degrees(relative_angle_rad)

    def update_current_offset(self):
        if not self.waypoint_buffer:
            return
        waypoint = self.waypoint_buffer[0]
        ego_location = self.location.x, self.location.y
        waypoint_location = waypoint.transform.location.x, waypoint.transform.location.y
        waypoint_yaw_rad = math.radians(waypoint.transform.rotation.yaw)
        cos_yaw, sin_yaw = math.cos(waypoint_yaw_rad), math.sin(waypoint_yaw_rad)
        normal_vector = -sin_yaw, cos_yaw
        vector_waypoint_to_ego = map(lambda coord1, coord2: coord1 - coord2, ego_location, waypoint_location)
        self.current_offset = sum(map(lambda coord1, coord2: coord1 * coord2, vector_waypoint_to_ego, normal_vector))
        if abs(self.current_offset) > 0.1:
            pass

    def check_traffic_light(self):
        if not self.config["ignore_traffic_light"]:
            if self.perception.is_traffic_light_red():
                self.target_speed = self.speed*0.3
                self.use_car_following = True
                return
        if self.vehicle.is_at_traffic_light():
            # self.target_speed = min(self.target_speed, 10)
            self.use_car_following = True
        else:
            pass

    def get_plt_info(self):
        # vehicle_types = self.config["topology"].keys() if self.config["topology"].values != -1
        # example vehicle_info = {"LV": {"speed": 10, "acc": 1, "x": 1, "y": 1, "yaw": 1, "state": 1}}
        vehicle_types = [
            key for key, value in self.config["topology"].items() if value != -1]
        vehicle_info = {}
        for vehicle_type in vehicle_types:
            vehicle_info[vehicle_type] = self.communi_agent.rec_obj(
                vehicle_type)
            if not vehicle_info[vehicle_type]:
                print(vehicle_info[vehicle_type], self.id)

        # print(vehicle_info)
        return vehicle_info
