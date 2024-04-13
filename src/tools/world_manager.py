import carla
import sys
import logging
import os
import carla
import random
from view.debug_manager import set_bird_view


class WorldManager:
    def __init__(self, config):
        self.config = config
        self.client = self.create_client()
        self.world = self.create_world()
        self.map = self.world.get_map()
        self.traffic_agent = self.set_traffic_agent()
        self.set_weather()
        self.set_world_parameter()
        self.cache_spawn_points()

    def set_traffic_agent(self):
        traffic_agent = self.client.get_trafficmanager()
        traffic_agent.set_synchronous_mode(False)
        traffic_agent.set_hybrid_physics_radius(
            self.config["hybrid_physics_radius"])
        traffic_agent.set_respawn_dormant_vehicles(True)
        traffic_agent.set_boundaries_respawn_dormant_vehicles(50, 800)
        return traffic_agent

    def create_client(self):
        client = carla.Client("localhost", self.config["carla_port"])
        client.set_timeout(self.config["carla_timeout"])
        return client

    def need_change_map(self, map_name, world):
        return str(world.get_map().name)[-2:] != map_name[-2:]

    def create_world(self):
        map_name = self.config.get("map_path") if self.config.get("is_custum_map") else self.config["map_name"]
        current_world = self.client.get_world()
        if self.need_change_map(map_name, current_world):
            if self.config["is_custum_map"]:
                # return current_world
                with open(self.config["map_path"], encoding='utf-8') as od_file:
                    try:
                        xodr_data = od_file.read()
                        # xodr_data = carla.Osm2Odr.convert(od_file.read())
                    except OSError:
                        print('file could not be read.')
                        sys.exit()
                logging.info('Converting OSM data to opendrive')
                logging.info('load opendrive map.')
                return self.client.generate_opendrive_world(
                    xodr_data, 
                    carla.OpendriveGenerationParameters(
                        vertex_distance=2.0,
                        max_road_length=19500.0,
                        wall_height=0.0,
                        additional_width=3.6,
                        smooth_junctions=True,
                        enable_mesh_visibility=True
                    )
                )
            else:
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
        settings.synchronous_mode = False
        settings.actor_active_distance = self.config["actor_active_distance"]
        settings.no_rendering_mode = self.config["no_rendering_mode"]
        settings.tile_stream_distance = self.config["tile_stream_distance"]
        settings.fixed_delta_seconds = self.config["fixed_delta_seconds"]
        self.world.apply_settings(settings)

    def cache_spawn_points(self):
        filepath = os.path.join("cache/sp_points", f"{self.map.name.split('/')[-1]}.csv")
        waypoints = [p.transform for p in self.map.generate_waypoints(5.0)]
        set_bird_view(self.world, waypoints[0].location)
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, "w") as file:
            file.writelines(map(lambda wp: f"{wp.location.x},{wp.location.y},{wp.location.z},{wp.rotation.yaw},{wp.rotation.pitch},{wp.rotation.roll}\n", waypoints))
        return waypoints

    def get_world(self):
        return self.world

    def get_traffic_manager(self):
        return self.traffic_agent
