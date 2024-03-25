from agent.base_vehicle import BaseVehicle
from util import  time_const,  clean_up, handle_exception
from view.debug_manager import draw_waypoints_arraw, draw_transforms, set_bird_view

from perception.sensor_manager import TruckSensorManager
from plan.cacc_planner import CACCPlanner
import carla
import logging
import time


class TruckVehicleAgent(BaseVehicle):
    def __init__(self, config):
        self.config = config
        BaseVehicle.__init__(
            self, self.config)


    def run(self):
        @time_const(fps=self.config["fps"])
        def run_step():
            obs = self.communi_agent.rec_obj("router")
            self.local_planner.run_step(obs)
        self.init_vehicle()
        # self.vehicle.set_target_velocity(carla.Vector3D(20, 0, 0))
        # self.trailer.set_target_velocity(carla.Vector3D(20, 0, 0))

        try:
            while True:
                if self.vehicle.attributes["role_name"] == "p_0":
                    set_bird_view(self.world, self.vehicle.get_location(),80,-100,-150,carla.Rotation(-25,90 , 0))
                    # set_bird_view(self.world, self.vehicle.get_location())
                run_step()
        except Exception as e:
            handle_exception(e)
            clean_up(self.world)
            self.close_agent()
            exit()


    def set_communi_agent(self):
        self.communi_agent.init_subscriber("router",
                                           self.config["traffic_agent_port"])
        topology = self.config["topology"]
        for key, index in topology.items():
            if key in ["index", "len"]:
                continue
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
            self.vehicle , self.trailer = vehicle, trailer
        except Exception as e:
            logging.error(f"create ego vehicle error:{e}")
            raise

    def get_vehicle_info(self):
        vehicle_bb = self.vehicle.bounding_box.extent
        trailer_bb = self.trailer.bounding_box.extent
        self.vehicle_info = {
            "width": vehicle_bb.y, 
            "length": vehicle_bb.x, 
            "height": vehicle_bb.z, 
            "mass": self.vehicle.get_physics_control().mass, 
            "trailer_width": trailer_bb.y, 
            "trailer_length": trailer_bb.x, 
            "trailer_height": trailer_bb.z, 
            "trailer_mass": self.trailer.get_physics_control().mass
        }
    def create_sensor_impl(self):
        self.sensor_manager = TruckSensorManager(
            self.world, self.vehicle, self.trailer, self.vehicle_info, self.config)
    def create_local_planner_impl(self):
        self.local_planner = CACCPlanner(self.world, self.map,self.start_point,self.end_point, self.vehicle, self.vehicle_info,self.config, self.global_route_planner, self.controller, self.sensor_manager, self.communi_agent)


