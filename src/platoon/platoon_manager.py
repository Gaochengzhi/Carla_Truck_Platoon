import carla
import logging
import random
import time


class Platoon():
    def __init__(self,world,client,filtered_waypoints,config):
        self.world = world
        self.client = client
        for platoon in config["platoon_list"]:
            plt_id = platoon["Platoon"]["plt_id"] 
            start_point = platoon["Platoon"]["start_point"]
            plt_size = platoon["Platoon"]["plt_size"]
            location = carla.Location(x=start_point[0], y=start_point[1], z=start_point[2])

# Create a Rotation object for the orientation
            rotation = carla.Rotation(pitch=start_point[3], yaw=start_point[4], roll=start_point[5])

            # Create the Transform object using the Location and Rotation
            spawn_point = carla.Transform(location, rotation)
            for i in range(plt_size):
                self.create_vehicle(world,client,filtered_waypoints,plt_id,start_point,config,spawn_point)
                forwardVector = spawn_point.get_forward_vector() * 15
                spawn_point.location -= forwardVector
                time.sleep(2.0)
    def create_vehicle(self,world,client,filtered_waypoints,plt_id,start_point,config,spawn_point):
        blueprintTruck = world.get_blueprint_library().filter("daf")[0]
        blueprintTrailer = world.get_blueprint_library().filter("trailer")[0]
        blueprintTrailer.set_attribute('role_name', 'hero-trailer')
        self.world.try_spawn_actor(blueprintTruck,spawn_point)
        forwardVector = spawn_point.get_forward_vector() * 3.3
        spawn_point.location -= forwardVector
        self.world.tick()
        self.world.tick()
        time.sleep(2.0)
        self.world.spawn_actor(blueprintTrailer, spawn_point )
        self.world.tick()







