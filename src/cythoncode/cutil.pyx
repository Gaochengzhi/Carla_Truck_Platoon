# util.pyx
import numpy as np
cimport numpy as np
from libc.math cimport sin, cos, sqrt, acos 
cimport cython

ctypedef np.float32_t FLOAT32_t
cdef inline float radians(float degrees):
    return degrees * (3.14159265 / 180.0)

cdef inline float degrees(float radians):
    return radians * (180.0 / 3.14159265)

def is_within_distance_obs(np.ndarray[FLOAT32_t, ndim=1] ego_location, 
                           np.ndarray[FLOAT32_t, ndim=1] target_location,
                           float max_distance=60, 
                           float ego_speed=0, 
                           float target_velocity=0, 
                           float ego_yaw=0, 
                           angle_interval=None):
    cdef np.ndarray[FLOAT32_t, ndim=1] target_vector
    cdef FLOAT32_t norm_target
    cdef np.ndarray[FLOAT32_t, ndim=1] forward_vector
    cdef FLOAT32_t angle

    
    # Calculate vector
    target_vector = target_location - ego_location  
    norm_target = sqrt(target_vector[0]**2 + target_vector[1]**2)
    
    # Distance checks
    if norm_target < 0.01 or norm_target > max_distance:
        return False
        
    # Calculate angle  
    forward_vector = get_forward_vector(ego_yaw)
    angle = degrees(acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1.0, 1.0)))
    
    # Check angle
    if (angle_interval is None or (angle >= angle_interval[0] and angle <= angle_interval[1])) and target_velocity <= ego_speed:
        return True
    else:
        return False

cdef np.ndarray[FLOAT32_t, ndim=1] get_forward_vector(float yaw):
    cdef FLOAT32_t rad = radians(yaw)
    return np.array([cos(rad), sin(rad)], dtype=np.float32)



@cython.boundscheck(False)
@cython.wraparound(False)
def compute_magnitude_angle(float target_location_x, float target_location_y,
                            float current_location_x, float current_location_y,
                            float orientation):
    """
    Compute relative angle and distance between a target_location and a current_location
    :param target_location_x: x-coordinate of the target object
    :param target_location_y: y-coordinate of the target object
    :param current_location_x: x-coordinate of the reference object
    :param current_location_y: y-coordinate of the reference object
    :param orientation: orientation of the reference object (in degrees)
    :return: a tuple composed by the distance to the object and the angle between both objects (in degrees)
    """
    cdef float dx = target_location_x - current_location_x
    cdef float dy = target_location_y - current_location_y
    cdef float norm_target = sqrt(dx ** 2 + dy ** 2)

    cdef float rad_orientation = radians(orientation)
    cdef float cos_orientation = cos(rad_orientation)
    cdef float sin_orientation = sin(rad_orientation)

    cdef float dot_product = dx * cos_orientation + dy * sin_orientation
    cdef float angle_rad = acos(np.clip(dot_product / norm_target, -1.0, 1.0))
    cdef float angle_deg = degrees(angle_rad)

    return (norm_target, angle_deg)