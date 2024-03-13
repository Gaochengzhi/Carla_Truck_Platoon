from agent.baseAgent import BaseAgent
from data.commuicate_manager import CommuniAgent
from util import connect_to_server, spawn_vehicle, time_const, is_within_distance, compute_distance, log_time_cost, txt_to_points
from view.debug_manager import draw_waypoints_arraw, draw_transforms, set_bird_view

from perception.sensor_manager import SensorManager
# from navigation.router_baseline import GlobalRoutePlanner
from cythoncode.router_baseline import GlobalRoutePlanner
# from navigation.controller_baseline import VehiclePIDController
from cythoncode.controller_baseline import VehiclePIDController
from plan.planer_baseline import FrenetPlanner
import carla
import logging
import time
import math
import random


class TruckVehicleAgent(BaseAgent):
    def __init__(self, config):
        self.config = config
        BaseAgent.__init__(
            self, self.config["name"], self.config["port"])

    def run(self):
        @time_const(fps=self.config["fps"])
        def run_step(world, control):
            obs = self.communi_agent.rec_obj("router")
            self.local_planner.run_step(obs, self.state)

        client, world = connect_to_server(1000, 2000)
        map = world.get_map()
        self.start_agent()
        self.set_communi_agent()
        self.start_point, self.end_point = self.get_navi_pos(map)
        self.vehicle, self.trailer = self.create_vehicle(
            world, self.start_point)
        self.state = "acc" if self.config["topology"]["LV"] == -1 else "cacc"
        self.vehicle_info = self.init_vehicle_info()
        print(f"vehicle_info:{self.vehicle_info}")
        self.sensor_manager = SensorManager(
            world, self.vehicle, self.vehicle_info, self.config)
        self.controller = VehiclePIDController(self.vehicle)
        self.global_route_planner = GlobalRoutePlanner(
            world.get_map(), sampling_resolution=3)
        self.global_router_waypoints = [x[0] for x in self.global_route_planner.trace_route(
            self.start_point.location, self.end_point.location)]
        self.local_planner = FrenetPlanner(
            world, map, self.global_router_waypoints, self.vehicle, self.config, self.controller, self.sensor_manager, self.communi_agent)
        control = carla.VehicleControl()
        try:
            while True:
                if self.vehicle.attributes["role_name"] == "p_0":
                    set_bird_view(world, self.vehicle.get_location(), 170)
                run_step(world, control)
        except Exception as e:
            logging.error(f"ego vehicle agent error:{e}")
            print(e.__traceback__.tb_frame.f_globals["__file__"])
            print(e.__traceback__.tb_lineno)
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
            self.close_agent()
            exit()

    def get_navi_pos(self, map):
        self.waypoints = map.generate_waypoints(5.0)
        start_point = self.waypoints[self.config["start_point"]].transform
        start_point.location.z += 2.0
        end_point = self.waypoints[self.config["end_point"]].transform
        return start_point, end_point

    # @log_time_cost(name="ego_vehicle_agent")
    def set_communi_agent(self):
        self.communi_agent.init_subscriber("router",
                                           self.config["traffic_agent_port"])

        topology = self.config["topology"]
        for key, index in topology.items():
            if index == -1:
                continue
            sub_port = self.config["base_port"] + index
            self.communi_agent.init_subscriber(key, sub_port)

    def create_vehicle(self, world, start_point, ego_vehicle_type="daf"):
        try:
            blueprintTruck = world.get_blueprint_library().filter(ego_vehicle_type)[
                0]
            blueprintTruck.set_attribute('role_name', self.name)
            blueprintTrailer = world.get_blueprint_library().filter("trailer")[
                0]
            blueprintTrailer.set_attribute('role_name', self.name+'trailer')
            trailer = world.spawn_actor(blueprintTrailer, start_point)
            forwardVector = start_point.get_forward_vector() * 5.0
            start_point.location += forwardVector
            vehicle = world.spawn_actor(blueprintTruck, start_point)
            return vehicle, trailer
        except Exception as e:
            logging.error(f"create ego vehicle error:{e}")
            raise

    def init_vehicle_info(self):
        v_length = self.vehicle.bounding_box.extent.x
        v_widht = self.vehicle.bounding_box.extent.y
        v_high = self.vehicle.bounding_box.extent.z
        v_mass = self.vehicle.get_physics_control().mass
        t_length = self.trailer.bounding_box.extent.x
        t_widht = self.trailer.bounding_box.extent.y
        t_high = self.trailer.bounding_box.extent.z
        t_mass = self.trailer.get_physics_control().mass
        return {"width": v_widht, "length": v_length, "height": v_high, "mass": v_mass, "trailer_wdith": t_widht, "trailer_length": t_length, "trailer_height": t_high, "trailer_mass": t_mass}

    def get_vehicle(self):
        return self.vehicle
