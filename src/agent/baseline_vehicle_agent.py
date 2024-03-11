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


class BaselineVehicleAgent(BaseAgent):
    def __init__(self, config):
        self.config = config
        BaseAgent.__init__(
            self, self.config["name"], self.config["port"])
        self.count = 0

    def run(self):
        @time_const(fps=self.config["fps"])
        def run_step(world, control):
            obs = self.communi_agent.rec_obj("router")
            state_id = self.local_planner.run_step(obs, control)

        client, world = connect_to_server(1000, 2000)
        map = world.get_map()
        self.start_agent()
        self.set_communi_agent()
        self.start_point, self.end_point = self.get_navi_pos(world)
        self.vehicle = self.create_vehicle(world, self.start_point,
                                           self.config["type"])

        self.vehicle_info = self.init_vehicle_info()
        self.sensor_manager = SensorManager(
            world, self.vehicle, self.vehicle_info, self.config)
        self.controller = VehiclePIDController(self.vehicle)
        self.global_route_planner = GlobalRoutePlanner(
            world.get_map(), sampling_resolution=3)
        self.global_router_waypoints = [x[0] for x in self.global_route_planner.trace_route(
            self.start_point.location, self.end_point.location)]
        self.local_planner = FrenetPlanner(
            world, map, self.global_router_waypoints, self.vehicle, self.config, self.controller, self.sensor_manager)
        control = carla.VehicleControl()
        # debug
        set_bird_view(world, self.start_point.location, 80)
        # draw_waypoints_arraw(
        #     world, self.global_router_waypoints, 1, life_time=100)
        try:
            while True:
                if self.vehicle.attributes["role_name"] == "emergency":
                    set_bird_view(world, self.vehicle.get_location(), 80)
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

    def get_navi_pos(self, world):
        self.waypoints = world.get_map().get_spawn_points()
        start_point = self.waypoints[self.config["start_point"]]
        end_point = self.waypoints[self.config["end_point"]]
        return start_point, end_point

    # @log_time_cost(name="ego_vehicle_agent")
    def set_communi_agent(self):
        self.communi_agent.init_subscriber("router",
                                           self.config["traffic_agent_port"])

    def create_vehicle(self, world, start_point, ego_vehicle_type):
        try:
            spawn_actor = spawn_vehicle(
                world, ego_vehicle_type, start_point, hero=True, name=self.config["name"])
            while spawn_actor is None:
                logging.info(
                    f"spawn_actor{ego_vehicle_type} failed, trying another start point...")
                start_point = random.choice(self.waypoints)
                spawn_actor = spawn_vehicle(
                    world, ego_vehicle_type, start_point, hero=True, name=self.config["name"])
            return spawn_actor
        except Exception as e:
            logging.error(f"create ego vehicle error:{e}")
            raise

    def init_vehicle_info(self):
        v_length = self.vehicle.bounding_box.extent.x
        v_widht = self.vehicle.bounding_box.extent.y
        v_high = self.vehicle.bounding_box.extent.z
        mass = self.vehicle.get_physics_control().mass
        return {"width": v_widht, "length": v_length, "height": v_high, "mass": mass}

    def get_trajection(self):
        return self.trajection

    def get_vehicle(self):
        return self.vehicle
