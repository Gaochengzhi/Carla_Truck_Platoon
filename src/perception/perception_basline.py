import carla
import logging
from util import compute_3D21d, log_time_cost


class FakePerception:
    def __init__(self, vehicle, config):
        self._fake_perception = True
        self.vehicle = vehicle
        self.config = config

    def get_road_edge(self, waypoint, oppsite=False):
        lane_width = waypoint.lane_width
        current_lane_type = waypoint.lane_type
        # Initialize offsets
        left_offset = 0
        right_offset = 0

        # Check for multiple lanes on the left
        left_lane = waypoint.get_left_lane()
        while left_lane and left_lane.lane_type == current_lane_type and left_lane.lane_id * waypoint.lane_id > 0:
            left_offset += left_lane.lane_width
            left_lane = left_lane.get_left_lane()

        # Check for multiple lanes on the right
        right_lane = waypoint.get_right_lane()
        while right_lane and right_lane.lane_type == current_lane_type and right_lane.lane_id * waypoint.lane_id > 0:
            right_offset += right_lane.lane_width
            right_lane = right_lane.get_right_lane()

        # Calculate distances to road edges
        distance_to_left_side = max(0, lane_width / 2 + left_offset)
        distance_to_right_side = max(0, lane_width / 2 + right_offset)

        # Return distances with a slight adjustment (if needed)
        return distance_to_left_side - 1, distance_to_right_side - 1

    def is_traffic_light_red(self):
        if self.vehicle.is_at_traffic_light():
            traffic_light = self.vehicle.get_traffic_light()
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                return True
        return False

    def get_ego_vehicle_info(self):
        location = self.vehicle.get_location()
        velocity = self.vehicle.get_velocity()
        transform = self.vehicle.get_transform()
        acceleration = self.vehicle.get_acceleration()
        speed = compute_3D21d(velocity)
        accel = compute_3D21d(acceleration)
        return location, transform, speed, accel
