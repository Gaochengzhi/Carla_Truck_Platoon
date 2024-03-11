import carla
import math
import time


class DebugManager:
    def __init__(self, world, urban_waypoints, config):
        self.world = world
        self.debug = world.debug


def update_view(world, x, y, z, rotation=carla.Rotation(-90, 0, 0)):
    spectator = world.get_spectator()
    spectator.set_transform(
        carla.Transform(
            carla.Location(x, y, z),
            rotation
        )
    )


def set_bird_view(world, location, high):
    update_view(
        world,
        location.x,
        location.y,
        location.z + high
    )


def draw_waypoints_arraw(world, waypoints, z=2, color=carla.Color(255, 0, 0), life_time=0.1):
    """
    Draw a list of waypoints at a certain height given in z.

        :param world: carla.world object
        :param waypoints: list or iterable container with the waypoints to draw
        :param z: height in meters
    """
    for wpt in waypoints:
        wpt_t = wpt.transform
        begin = wpt_t.location + carla.Location(z=z)
        angle = math.radians(wpt_t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=life_time)


def draw_waypoints(world, waypoints, z=2, color=carla.Color(255, 0, 0), size=0.09, life_time=0.1):
    """
    draw a list of waypoints
    """
    for waypoint in waypoints:
        world.debug.draw_point(
            waypoint.transform.location, size=size, color=color, life_time=life_time
        )


def draw_xy(world, xs, ys, color=carla.Color(255, 0, 0), size=0.09, life_time=0.1):
    for x, y in zip(xs, ys):
        carla_location = carla.Location(x=x, y=y, z=3)
        world.debug.draw_point(
            carla_location, size=size, color=color, life_time=life_time
        )


def draw_list(world, list_xy, color=carla.Color(255, 0, 0), size=0.09, life_time=0.1):
    for point in list_xy:
        carla_location = carla.Location(x=point[0], y=point[1], z=2)
        world.debug.draw_point(
            carla_location, size=size, color=color, life_time=life_time
        )


def draw_future_locations(world, future_list, life_time=1):
    for future_location in future_list:
        carla_location = carla.Location(
            x=future_location["location"].x, y=future_location["location"].y, z=future_location["location"].z)
        world.debug.draw_point(
            carla_location,
            size=0.2,  # Size of the point
            color=carla.Color(255, 0, 0),
            life_time=life_time,
        )


def draw_transforms(world, transforms, color=carla.Color(255, 0, 0), size=0.09, life_time=0.1):
    """
    draw a list of transforms
    """
    for transform in transforms:
        world.debug.draw_point(
            transform.location, size=size, color=color, life_time=life_time
        )


def draw_transforms_with_index(world, transforms, color=carla.Color(255, 0, 0), size=0.09, life_time=0.1):
    """
    draw a list of transforms with text
    """
    for index, transform in enumerate(transforms):
        world.debug.draw_point(
            transform.location, size=size, color=color, life_time=life_time
        )
        world.debug.draw_string(
            transform.location, str(index), draw_shadow=False, color=color, life_time=life_time
        )


def draw_strings(world, strings, location, color=carla.Color(255, 0, 0), life_time=0.09):
    for string in strings:
        world.debug.draw_string(
            location, string, draw_shadow=False, color=color, life_time=life_time
        )
