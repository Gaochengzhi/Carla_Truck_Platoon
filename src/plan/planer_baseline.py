import time
import functools
import numpy as np
import math
import random
import sys
import carla
from util import compute_distance, log_time_cost, compute_3D21d, compute_distance2D, interpolate_points, time_const
from cythoncode.cutil import is_within_distance_obs, compute_magnitude_angle
from view.debug_manager import draw_waypoints, draw_list
from scipy.interpolate import splprep, splev
from plan.carFollowingModel import IDM, BDL_Controller
from perception.perception_basline import FakePerception
import logging
from pyinstrument import Profiler


class FrenetPlanner():
    def __init__(self, world, map, router_waypoints, vehicle, config, controller, sensor_manager, commuic_agent):
        self.global_waypoints = router_waypoints
        self.world = world
        self.map = map
        self.config = config
        self.state = "acc" if self.config["topology"]["LV"] == -1 else "cacc"
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
        self.cacc_model = IDM(
        ) if self.config["topology"]["LV"] == -1 else BDL_Controller()
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
    def run_step(self, obs,  state):
        try:
            # self.profiler.start()
            self.ego_state_update()
            self.obstacle_update(obs)
            self.check_waypoints()
            if len(self.trajectories) < 1:
                return
            self.check_trajectories()
            if state == "cacc":
                front_dis = 30
                back_dis = 30
                plt_info_lv = self.plt_info.get("LV")
                plt_info_fv = self.plt_info.get("FV")
                plt_info_rv = self.plt_info.get("RV")
                leading_v = plt_info_lv.get("speed") if plt_info_lv else None
                self.front_xy = plt_info_fv.get("xy") if plt_info_fv else None
                back_xy = plt_info_rv.get("xy") if plt_info_rv else None
                ego_xy = [self.location.x, self.location.y]
                if self.front_xy:
                    front_dis = compute_distance2D(ego_xy, self.front_xy)
                if back_xy:
                    back_dis = compute_distance2D(ego_xy, back_xy)
                else:
                    back_dis = 30
                if front_dis and leading_v:
                    # print(front_dis, self.vehicle.attributes["role_name"])
                    self.target_speed = self.cacc_model.calc_speed(
                        front_dis, back_dis, self.speed, leading_v)
                else:
                    print("lost", self.step,
                          self.vehicle.attributes["role_name"])
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
            if len(self.trajectories) < 1:
                return
            # DEBUG
            draw_waypoints(self.world, self.waypoint_buffer,
                           z=2, life_time=0.05, size=0.05)
            draw_list(self.world, self.trajectories, size=0.05,
                      color=carla.Color(0, 250, 123), life_time=0.05)

            if self.use_car_following:
                leading_vehicle = self.sensor_manager.radar_res["front"]
                leading_obs = self.sensor_manager.obstacle
                if leading_vehicle:
                    front_v = leading_vehicle[1] + self.speed
                    distance_s = leading_vehicle[0]
                    a = self.car_following_model.calc_acc(
                        front_v, distance_s, self.speed)
                    self.target_speed = max(0, self.speed + a)
                elif leading_obs:
                    distance_s = leading_obs.distance
                    front_v = leading_obs.velocity
                    a = self.car_following_model.calc_acc(
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
            if self.step >= 1000:
                self.target_speed = 15
            target_waypoint = carla.Transform(
                carla.Location(x=self.trajectories[0][0], y=self.trajectories[0][1]))
            control = self.controller.run_step(
                self.target_speed, target_waypoint)

            self.vehicle.apply_control(control)
            self.step += 1
            self.change_back_step += 1
            # self.profiler.stop()
            # self.profiler.print(show_all=True)

        except Exception as e:
            logging.error(f"plan run_step error: {e}")
            logging.error(e.__traceback__.tb_frame.f_globals["__file__"])
            logging.error(e.__traceback__.tb_lineno)
            pass

    def check_collison_and_change_lanes(self):
        if self.step % self.wait_step == 0:
            if self.check_collision():
                self.adjust_trajectories()
                self.wait_step += 1
            else:
                self.use_car_following = False
                self.wait_step = 4
                self.target_speed = min(
                    self.max_speed, self.speed*1.5)
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
                # self.vehicle.destroy()
                logging.info("finish the route!")
                # exit()
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
        left_options = np.arange(self.target_offset, -self.left_side - 1, -1)
        right_options = np.arange(
            self.target_offset+1, self.right_side, 1)
        traj_adjust_options = np.concatenate(
            (left_options, right_options))
        collision_info = []
        """
        collision_info: [(offset, highest_velocity, index)]
        """
        for offset in traj_adjust_options:
            self.update_trajectories(offset)
            collision_result = self.check_collision()
            if collision_result:
                # Find the highest velocity among collisions for this offset
                highest_velocity = max(collision_result, key=lambda x: x[1])[1]
                nearest_index = min(collision_result, key=lambda x: x[2])[2]
                collision_info.append(
                    (offset, highest_velocity, nearest_index))
            else:
                # Found a collision-free offset, apply it and return
                # if self.change_back_step > 3 * self.config["fps"]:
                self.change_back_step = 0
                if abs(offset-self.current_offset) > 4:
                    self.target_offset - 3 if offset > 0 else self.target_offset + 3
                    self.use_car_following = True
                    return False
                else:
                    self.target_offset = offset
                return True
        # If here, no collision-free offset was found; decide based on obstacle velocities and distances
        if collision_info:
            # Choose the offset related to the nearest highest-velocity obstacle
            chosen_offset, chosen_velocity, _ = max(
                collision_info, key=lambda x: (x[1], -x[2]))
            self.use_car_following = True
        # Check if the chosen velocity is significantly higher than the current velocity
            velocity_threshold = 2  # Adjust this value according to your preference
            if chosen_velocity >= self.speed + velocity_threshold:
                if abs(chosen_offset-self.current_offset) > 4:
                    self.target_offset - 3 if offset > 0 else self.target_offset + 3
                else:
                    self.target_offset = offset
            #     self.target_offset = chosen_offset
            #     self.update_trajectories(chosen_offset)
            #     return False

    def check_update_waypoints(self):
        if len(self.waypoint_buffer) < 1:
            self.update_waypoint_buffer()
        if len(self.trajectories) < 1:
            self.update_waypoint_buffer()
            self.update_trajectories(self.target_offset)

    # @log_time_cost(name="update_trajectories")
    def update_trajectories(self, offset=0):
        xy_list = [[waypoint.transform.location.x, waypoint.transform.location.y]
                   for waypoint in self.waypoint_buffer]
        lenxy = len(xy_list)
        xy_list = np.array(xy_list)
        if lenxy > 6:
            tck, u = splprep(xy_list.T, s=8, k=5)
        elif lenxy > 1:
            tck, u = splprep(xy_list.T, s=8, k=lenxy-1)
        else:
            self.trajectories = xy_list.tolist()
            return
        u_new = np.linspace(u.min(), u.max(), lenxy * 2)
        x_fine, y_fine = splev(u_new, tck)
        interpolated_waypoints = np.column_stack([x_fine, y_fine])
        if abs(offset) > 0.1:
            directions = np.diff(interpolated_waypoints, axis=0)
            normals = np.array([-directions[:, 1], directions[:, 0]]).T
            magnitudes = np.sqrt(
                normals[:, 0]**2 + normals[:, 1]**2).reshape(-1, 1)
            normals = normals / magnitudes
            offset_vectors = normals * offset
            offset_vectors = np.vstack([[offset_vectors[0]], offset_vectors])
            interpolated_waypoints += offset_vectors
            number_to_drop = min(
                len(interpolated_waypoints)-1, int(self.speed))
            interpolated_waypoints = interpolated_waypoints[number_to_drop:]
        self.trajectories = interpolated_waypoints.tolist()

    def check_collision(self):
        """
        collision_info: [obstacle, velocity, index(distence)]
        """
        start_location = [self.location.x, self.location.y]
        end_location = self.trajectories[0]
        interpolated_points = interpolate_points(
            start_location, end_location, 2)
        interpolated_points.pop(0)
        self.trajectories = interpolated_points + self.trajectories
        draw_list(self.world, interpolated_points, size=0.1,
                  color=carla.Color(110, 25, 144), life_time=0.15)
        collision_info = []
        for ob in self.obs_list:
            for index, point in enumerate(self.trajectories):
                if compute_distance2D((point[0], point[1]), (ob[0], ob[1])) < self.detect_range:
                    collision_info.append((ob, ob[2], index))
                    break
        return collision_info if collision_info else False

    def update_waypoint_buffer(self):
        num_push = min(int(self.speed/1.5)+3,
                       len(self.global_waypoints), 30)
        self.waypoint_buffer = self.global_waypoints[:num_push]
        if num_push < 3:
            return
        curvature = 0
        for i in range(1, num_push - 1):
            x1, y1 = self.waypoint_buffer[i -
                                          1].transform.location.x, self.waypoint_buffer[i-1].transform.location.y
            x2, y2 = self.waypoint_buffer[i].transform.location.x, self.waypoint_buffer[i].transform.location.y
            x3, y3 = self.waypoint_buffer[i +
                                          1].transform.location.x, self.waypoint_buffer[i+1].transform.location.y
            # Vector from the first to the second point
            dx1, dy1 = x2 - x1, y2 - y1
            # Vector from the second to the third point
            dx2, dy2 = x3 - x2, y3 - y2
            # Normalize vectors
            mag1 = (dx1**2 + dy1**2)**0.5
            mag2 = (dx2**2 + dy2**2)**0.5
            if mag1 > 0 and mag2 > 0:
                dx1, dy1 = dx1 / mag1, dy1 / mag1
                dx2, dy2 = dx2 / mag2, dy2 / mag2
                # Dot product between vectors (cosine of the angle)
                dot = dx1*dx2 + dy1*dy2
                # Update curvature (using 1 - dot to get a measure of deviation from straight line)
                curvature += (1 - dot)
        if curvature > 0.0001:
            self.target_speed = max(
                10, self.max_speed * math.exp(-6*curvature))
        else:
            self.target_speed = self.max_speed

    def obstacle_update(self, obs):
        self.obs_list = []
        if not obs:
            return
        obs_list = []
        ego_location = np.array(
            [self.transform.location.x, self.transform.location.y], dtype=np.float32)
        ego_yaw = self.transform.rotation.yaw
        for obs_info in obs:
            if obs_info["id"] != self.id:
                target_location = np.array([
                    obs_info["location"].x, obs_info["location"].y], dtype=np.float32)
                target_velocity = obs_info["velocity"]
                if is_within_distance_obs(ego_location, target_location, 50, self.speed, target_velocity, ego_yaw, [-65, 65]):
                    obs_list.append(
                        [obs_info["location"].x, obs_info["location"].y, obs_info["velocity"], obs_info["yaw"]])
                    obs_list.append(
                        [obs_info["flocation"].x, obs_info["flocation"].y, obs_info["fvelocity"], obs_info["yaw"]])

        self.communi_agent.send_obj({
            "speed": self.speed,
            "acc": self.acc,
            "xy": [self.location.x, self.location.y],
            "yaw": self.transform.rotation.yaw,
            "state": self.state
        })
        self.plt_info = self.get_plt_info()
        # else:
        #     if obs_info["except_v"]:
        #         self.max_speed = obs_info["except_v"]
        #     else:
        #         self.max_speed = self.config.get("max_speed", 30)
        #     if obs_info["except_offset"]:
        #         self.target_offset = - self.left_side - \
        #             0 if self.left_side < self.right_side else self.right_side + 1

        self.obs_list = obs_list

    def compute_brake(self, distance):
        brake = math.exp(-distance/10)
        return brake**0.4

    def ego_state_update(self):
        self.location, self.transform, self.speed, self.acc = self.perception.get_ego_vehicle_info()

    def get_relative_waypoint_angle(self):
        if self.waypoint_buffer:
            waypoint = self.waypoint_buffer[0]
            waypoint_yaw_rad = math.radians(waypoint.transform.rotation.yaw)
            ego_yaw_rad = math.radians(self.transform.rotation.yaw)
            relative_angle_rad = ego_yaw_rad - waypoint_yaw_rad
            relative_angle_rad = (relative_angle_rad +
                                  math.pi) % (2 * math.pi) - math.pi
            relative_angle_deg = math.degrees(relative_angle_rad)
            return relative_angle_deg
        return 0

    def update_current_offset(self):
        if self.waypoint_buffer:
            waypoint = self.waypoint_buffer[0]
            ego_location = self.location.x, self.location.y
            waypoint_location = waypoint.transform.location.x, waypoint.transform.location.y
            waypoint_yaw_rad = math.radians(waypoint.transform.rotation.yaw)
            cos_yaw, sin_yaw = math.cos(
                waypoint_yaw_rad), math.sin(waypoint_yaw_rad)
            normal_vector = -sin_yaw, cos_yaw
            vector_waypoint_to_ego = [
                coord1 - coord2 for coord1, coord2 in zip(ego_location, waypoint_location)]

            self.current_offset = sum(
                coord1 * coord2 for coord1, coord2 in zip(vector_waypoint_to_ego, normal_vector))
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
        return vehicle_info
