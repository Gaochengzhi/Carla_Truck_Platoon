import carla
import numpy as np
import weakref
from util import compute_3D21d
from view.debug_manager import draw_transforms
import math


class BaseSensorManager:
    def __init__(self, world, vehicle,trailer, vehicle_info, config):
        self.world = world
        self.vehicle = vehicle
        self.trailer = trailer
        self.vehicle_info = vehicle_info
        self.config = config
        self.radar_list = []
        self.radar_res = {
            radar_id: None for radar_id in ["front", "left", "right","rear"]
        }
        self.setup_truck_radars()
        self.add_obstacle_sensor()
        self.camera_queue = []
        self.radar_queue = []
        self.obstacle = None
        self.rear_obstacle = None


    def add_camera(self):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '192')
        camera_bp.set_attribute('image_size_y', '108')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))

        self.camera = self.world.spawn_actor(
            camera_bp, camera_transform, attach_to=self.vehicle)
        self.camera.listen(self.camera_callback)

    def camera_callback(self, data):
        pass

    def setup_truck_radars(self):
        radar_transforms = {
            "front": carla.Transform(carla.Location(x=self.vehicle_info["length"], z=self.vehicle_info["height"]), carla.Rotation(pitch=0)),
            "rear": carla.Transform(carla.Location(x=-self.vehicle_info["trailer_length"], z=self.vehicle_info["trailer_height"]), carla.Rotation(yaw=180, pitch=0)),
            "left": carla.Transform(carla.Location(y=-self.vehicle_info["width"], z=self.vehicle_info["height"]), carla.Rotation(yaw=-80, pitch=7)),
            "right": carla.Transform(carla.Location(y=self.vehicle_info["width"], z=self.vehicle_info["height"]), carla.Rotation(yaw=80, pitch=7))
        }

        radar_specs = {
            "front": {"h_fov": 5, "v_fov": 1, "range": 50, "attach_to": self.vehicle},
            "rear": {"h_fov": 5, "v_fov": 1, "range": 50, "attach_to": self.trailer},
            "left": {"h_fov": 120, "v_fov": 5, "range": 3, "attach_to": self.vehicle},
            "right": {"h_fov": 120, "v_fov": 5, "range": 3, "attach_to": self.vehicle}
        }

        self.radar_list = [self.add_radar(radar_id, **radar_specs[radar_id], radar_transform=radar_transforms[radar_id]) for radar_id in radar_specs]

    def add_radar(self, radar_id, h_fov, v_fov, radar_transform, points_per_second="1000", range="50", attach_to=None):
        weak_self = weakref.ref(self)
        radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', str(h_fov))
        radar_bp.set_attribute('vertical_fov', str(v_fov))
        radar_bp.set_attribute('points_per_second', str(points_per_second))
        radar_bp.set_attribute('range', str(range))
        radar = self.world.spawn_actor(
            radar_bp, radar_transform, attach_to=attach_to, attachment_type=carla.AttachmentType.Rigid)
        radar.listen(lambda data: self.radar_callback(
            weak_self, radar_id, data))
        return radar

    def add_obstacle_sensor(self):
        weak_self = weakref.ref(self)
        obstacle_bp = self.world.get_blueprint_library().find('sensor.other.obstacle')
        obstacle_bp.set_attribute('distance', '50')
        obstacle_bp.set_attribute("only_dynamics", str(False))
        # obstacle_bp.debug_linetrace = True
        front_obs_transform = carla.Transform(
            carla.Location(x=self.vehicle_info["length"]+5, z=self.vehicle_info["height"]))
        rear_obs_transform = carla.Transform(
            carla.Location(x=-self.vehicle_info["length"], z=self.vehicle_info["height"]))
        self.obstacle_sensor = self.world.spawn_actor(
            obstacle_bp, front_obs_transform, attach_to=self.vehicle, attachment_type=carla.AttachmentType.Rigid)
        self.back_obstacle_sensor = self.world.spawn_actor(
            obstacle_bp, front_obs_transform, attach_to=self.vehicle, attachment_type=carla.AttachmentType.Rigid)
        self.obstacle_sensor.listen(
            lambda event: self.obstacle_callback(weak_self, event))

    @staticmethod
    def obstacle_callback(weak_self, data):
        self = weak_self()
        if not self:
            return
        obs_loc = data.other_actor.get_location()
        obs_speed = compute_3D21d(data.other_actor.get_velocity())
        self.obstacle = Obstacle(obs_loc,
                                 data.distance, obs_speed)
        if data.other_actor.is_alive:
            data_transform = data.other_actor.get_transform()
            data_transform.location.z = 9
            draw_transforms(
                self.world, [data_transform], size=0.1, life_time=0.1 , color=carla.Color(155, 120, 100))

    @staticmethod
    def radar_callback(weak_self, radar_id, radar_data):
        self = weak_self()
        if not self or not radar_data:
            self.radar_res[radar_id] = None
            return

        self.radar_res[radar_id] = [(detect.depth, detect.velocity) for detect in radar_data][0]

        # current_rot = radar_data.transform.rotation
        # color = carla.Color(255, 0, 0)

        # for detect in radar_data:
        #     fw_vec = carla.Transform(
        #         carla.Location(), 
        #         carla.Rotation(
        #             pitch=current_rot.pitch + math.degrees(detect.altitude), 
        #             yaw=current_rot.yaw + math.degrees(detect.azimuth), 
        #             roll=current_rot.roll
        #         )
        #     ).transform(carla.Vector3D(x=detect.depth - 0.15))

        #     self.world.debug.draw_point(
        #         radar_data.transform.location + fw_vec,
        #         size=0.15,
        #         life_time=0.16,
        #         persistent_lines=False,
        #         color=color
        #     )


class Obstacle:
    def __init__(self, location, distance, velocity):
        self.location = location
        self.distance = distance
        self.velocity = velocity
