from agent.base_agent import BaseAgent
from data.commuicate_manager import CommuniAgent
from util import connect_to_server, spawn_vehicle, time_const, is_within_distance, compute_distance, log_time_cost, clean_up, handle_exception
from view.debug_manager import draw_waypoints_arraw, draw_transforms, set_bird_view

from perception.sensor_manager import BaseSensorManager
from navigation.router_baseline import GlobalRoutePlanner
# from cythoncode.router_baseline import GlobalRoutePlanner
from navigation.controller_baseline import VehiclePIDController
# from cythoncode.controller_baseline import VehiclePIDController
from plan.base_planer import BasePlanner
import logging
import random
import carla
import time

class BaseVehicle(BaseAgent):
    def __init__(self, config):
        self.config = config
        BaseAgent.__init__(
            self, self.config["name"], self.config["port"])

    def run(self):
        self.init_vehicle()
        @time_const(fps=self.config["fps"])
        def run_step():
            set_bird_view(self.world, self.vehicle.get_location())
            # set_bird_view(self.world, self.vehicle.get_location(), rotation=carla.Rotation(yaw=self.vehicle.get_transform().rotation.yaw,pitch=-90))
            obs = self.communi_agent.rec_obj("router")
            self.local_planner.run_step(obs)
        try:
            while True:
                run_step()
        except Exception as e:
            handle_exception(e)
            clean_up(self.world)
            self.close_agent()
            exit()

    def init_vehicle(self):
        _, self.world = connect_to_server(self.config["carla_timeout"], self.config["carla_port"])
        self.map = self.world.get_map()
        self.init_base_agent()
        self.get_start_end_point(self.map)
        self.create_vehicle(
            self.world, self.start_point)
        self.get_vehicle_info()
        self.set_communi_agent()
        self.create_sensor_impl()
        self.create_global_route_planner_impl()
        self.create_controller_impl()
        self.create_local_planner_impl()



    def create_vehicle(self, world, start_point, ego_vehicle_type="mini"):
        try:
            spawn_actor = spawn_vehicle(
                world, ego_vehicle_type, start_point, hero=True, name=self.config["name"])
            while spawn_actor is None:
                logging.info(
                    f"spawn_actor{ego_vehicle_type} failed, trying another start point...")
                start_point = random.choice(self.waypoints)
                spawn_actor = spawn_vehicle(
                    world, ego_vehicle_type, start_point, hero=True, name=self.config["name"])
            self.vehicle  = spawn_actor
        except Exception as e:
            logging.error(f"create ego vehicle error:{e}")
            raise


    def get_vehicle_info(self):
        vehicle_bb = self.vehicle.bounding_box.extent
        self.vehicle_info = {
            "width": vehicle_bb.y,
            "length": vehicle_bb.x,
            "height": vehicle_bb.z,
            "mass": self.vehicle.get_physics_control().mass,
        }

    def create_controller_impl(self):
        self.controller = VehiclePIDController(self.vehicle)
    def create_sensor_impl(self):
        self.sensor_manager = BaseSensorManager(
            self.world, self.vehicle, self.vehicle_info, self.config)

    def create_global_route_planner_impl(self):
        self.global_route_planner = GlobalRoutePlanner(
            self.map, sampling_resolution=3)

    def create_local_planner_impl(self):
        self.local_planner = BasePlanner(
            self.world, self.map,self.start_point,self.end_point, self.vehicle, self.vehicle_info,self.config, self.global_route_planner, self.controller, self.sensor_manager, self.communi_agent)
    def get_start_end_point(self, map):
        waypoints = map.generate_waypoints(5.0)
        start_point = waypoints[self.config["start_point"]].transform
        start_point.location.z += 1.0
        end_point = waypoints[self.config["end_point"]].transform
        self.start_point, self.end_point = start_point, end_point

    def set_communi_agent(self):
        self.communi_agent.init_subscriber("router",
                                           self.config["traffic_agent_port"])
    
