from util import get_vehicle_info, log_time_cost, compute_3D21d


class Location:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def compute_future_state(info, dt):
    dt = dt+5
    location = info['location']
    velocity = info['velocity']
    acceleration = info['acceleration']
    future_location = Location(
        x=location.x + velocity.x * dt + 0.5 * acceleration.x * dt ** 2,
        y=location.y + velocity.y * dt + 0.5 * acceleration.y * dt ** 2,
        z=location.z + velocity.z * dt + 0.5 * acceleration.z * dt ** 2
    )
    location = Location(
        x=location.x,
        y=location.y,
        z=location.z
    )
    future_velocity = Vector3D(
        x=velocity.x,
        y=velocity.y,
        z=velocity.z
    )
    future_state = {
        'id': info['id'],
        'location': location,
        'flocation': future_location,
        "velocity": compute_3D21d(velocity),
        'fvelocity': compute_3D21d(future_velocity),
        'yaw': info['transform'].rotation.yaw,
        "except_v": None,
        "except_offset": None

    }
    return future_state


def predict(world, vehicle, fps):
    info = get_vehicle_info(vehicle)
    future_state = compute_future_state(info, dt=1.0/fps)
    return future_state


class ObstacleVehicle:

    def __init__(self, vehicle, info):

        pass
