import carla
import time
import sys
import logging
import os
import random
from util import load_points_from_csv, spawn_vehicle, get_ego_vehicle
class WorldManager:
    def __init__(self, config):
        self.config = config
        self.client = self.create_client()
        self.world = self.create_world()
        self.map = self.world.get_map()
        self.traffic_agent = self.set_traffic_agent()
        self.set_weather()
        self.set_world_parameter()
        self._gen_filtered_points()

    def choose_a_point(self, waypoint_list):
        choosen_waypoint = random.choice(waypoint_list)
        waypoint_list.remove(choosen_waypoint)
        return choosen_waypoint

    def set_traffic_agent(self):
        traffic_agent = self.client.get_trafficmanager()
        traffic_agent.set_synchronous_mode(False)
        traffic_agent.set_hybrid_physics_radius(
            self.config["hybrid_physics_radius"])
        # for larger map
        traffic_agent.set_respawn_dormant_vehicles(True)
        traffic_agent.set_boundaries_respawn_dormant_vehicles(50, 800)
        return traffic_agent

    def create_client(self):
        client = carla.Client("localhost", self.config["carla_port"])
        client.set_timeout(self.config["carla_timeout"])
        return client

    def _compare_map(self, map_name, world):
        return str(world.get_map().name)[-2:] != map_name[-2:]

    def create_world(self):
        map_name = self.config["map_name"]
        if self.config["is_custum_map"]:
            data = None
            with open(self.config["map_path"], encoding='utf-8') as od_file:
                try:
                    data = od_file.read()
                except OSError:
                    print('file could not be readed.')
                    sys.exit()
            logging.info('Converting OSM data to opendrive')
            xodr_data = data
            logging.info('load opendrive map.')
            vertex_distance = 8.0  # in meters
            max_road_length = 9500.0  # in meters
            wall_height = 0.5      # in meters
            extra_width = 1.6      # in meters
            world = self.client.generate_opendrive_world(
                xodr_data, carla.OpendriveGenerationParameters(
                    vertex_distance=vertex_distance,
                    max_road_length=max_road_length,
                    wall_height=wall_height,
                    additional_width=extra_width,
                    smooth_junctions=True,
                    enable_mesh_visibility=True))
            return world

        current_world = self.client.get_world()
        if (
            self._compare_map(map_name, current_world)
        ):
            return self.client.load_world(map_name)
        else:
            return current_world

    def set_weather(self):
        clear_weather = carla.WeatherParameters(
            cloudiness=self.config["cloudiness"],
            precipitation=self.config["precipitation"],
            sun_altitude_angle=self.config["sun_altitude_angle"],
            sun_azimuth_angle=self.config["sun_azimuth_angle"],
        )
        self.world.set_weather(clear_weather)

    def set_world_parameter(self):
        settings = self.world.get_settings()
        settings.actor_active_distance = self.config["actor_active_distance"]
        settings.synchronous_mode = False
        settings.no_rendering_mode = self.config["no_rendering_mode"]
        settings.tile_stream_distance = self.config["tile_stream_distance"]
        settings.fixed_delta_seconds = self.config["fixed_delta_seconds"]
        self.world.apply_settings(settings)


    def _gen_filtered_points(self):
        map_name = self.map.name.split("/")[-1]
        cache_dir = "cache/sp_points"
        filename = f"{map_name}.csv"
        filepath = os.path.join(cache_dir, filename)
        distance_sps = self.map.generate_waypoints(5.0)
        filtered_points = []
        for p in distance_sps:
            filtered_points.append(p.transform)

        os.makedirs(cache_dir, exist_ok=True)
        with open(filepath, "w") as file:
            for sp in filtered_points:
                file.write(
                    f"{sp.location.x},{sp.location.y},{sp.location.z},{sp.rotation.yaw},{sp.rotation.pitch},{sp.rotation.roll}\n")
        return filtered_points

    def get_map(self):
        return self.map

    def get_world(self):
        return self.world

    def get_client(self):
        return self.client

    def get_traffic_manager(self):
        return self.traffic_agent
