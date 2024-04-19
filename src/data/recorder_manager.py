import random
import math
import os
import logging
import carla
from view.debug_manager import DebugManager as debug
from util import spawn_vehicle, connect_to_server, time_const, batch_process_vehicles, get_ego_vehicle, get_speed, log_time_cost, get_vehicle_info, init_data_file
import time
import csv


class DataRecorder():
    def __init__(
        self,
        config,
    ) -> None:
        self.config = config
        self.step = 0
        self.writer = self.run()

    def run_step(self, world):
        current_time = self.step
        try:
            batch_process_vehicles(
                world, self.write_vehicle_info, self.writer, time_step=current_time)
            self.step += 1
        except Exception as e:
            logging.error(e)

    def run(self):
        if not self.config["record"]:
            return
        self.fp, writer = init_data_file(
            self.config["data_folder_path"], "data.csv")
        writer.writerow([
            'time',
            'real_time',
            'vehicle_id',
            'location_x',
            'location_y',
            'location_z',
            'velocity_x',
            'velocity_y',
            'velocity_z',
            'acceleration_x',
            'acceleration_y',
            'acceleration_z',
            'angular_velocity_x',
            'angular_velocity_y',
            'angular_velocity_z',
            'rotation_roll',
            'rotation_pitch',
            'rotation_yaw',
            'control_brake',
            'control_throttle',
            'control_steer',
            # 'torque_curveX',
            # 'torque_curveY',
        ])
        return writer

    def write_vehicle_info(self, world, vehicle, writer, time_step):
        # Basic info
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        acceleration = vehicle.get_acceleration()
        angular_velocity = vehicle.get_angular_velocity()
        transform = vehicle.get_transform()
        control = vehicle.get_control()
        # Physics control info
        # physics_control = vehicle.get_physics_control()

        # # Extracting data from physics_control
        # torque_curve = [(point.x, point.y)
        #                 for point in physics_control.torque_curve]

        # Writing data to CSV within the locked context to ensure thread safety
        writer.writerow([
            time_step,
            time.time(),
            vehicle.attributes["role_name"],
            location.x, location.y, location.z,
            velocity.x, velocity.y, velocity.z,
            acceleration.x, acceleration.y, acceleration.z,
            angular_velocity.x, angular_velocity.y, angular_velocity.z,
            transform.rotation.roll, transform.rotation.pitch, transform.rotation.yaw,
            control.brake, control.throttle, control.steer,
        ])


    def close(self) -> None:
        self.fp.fflush()
        self.fp.close()
        time.sleep(1)
        return super().close()