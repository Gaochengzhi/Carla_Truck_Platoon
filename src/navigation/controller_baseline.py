# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains PID controllers to perform lateral and longitudinal control. """

import cvxopt
from collections import deque
import math
import numpy as np
import carla
from util import get_speed, log_time_cost


class VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers
    (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, vehicle, args_lateral={'K_P': 1.45,
                                              'K_I': 0.05, 'K_D': 0.2, 'dt': 0.1}, args_longitudinal={
        'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': 0.03}, offset=0, max_throttle=0.55, max_brake=0.9,
            max_steering=0.5):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller
        using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param offset: If different than zero, the vehicle will drive displaced from the center line.
        Positive values imply a right offset while negative ones mean a left one. Numbers high enough
        to cause the vehicle to drive through other lanes might break the controller.
        """

        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering

        self._vehicle = vehicle
        # self._world = self._vehicle.get_world()
        self.past_steering = self._vehicle.get_control().steer
        self._lon_controller = PIDLongitudinalController(
            self._vehicle, **args_longitudinal)
        # self._lat_controller = PIDLateralController(
        #     self._vehicle, offset, **args_lateral)
        self._lat_controller = MPCLateralController(self._vehicle)

    def run_step(self, target_speed, transform):
        acceleration = self._lon_controller.run_step(target_speed*3.6)
        current_steering = self._lat_controller.run_step(transform)
        control = carla.VehicleControl()
        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throt)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Steering regulation: changes cannot happen abruptly, can't steer too much.

        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering

        return control

    def change_longitudinal_PID(self, args_longitudinal):
        """Changes the parameters of the PIDLongitudinalController"""
        self._lon_controller.change_parameters(**args_longitudinal)

    def change_lateral_PID(self, args_lateral):
        """Changes the parameters of the PIDLateralController"""
        self._lat_controller.change_parameters(**args_lateral)

    def set_offset(self, offset):
        """Changes the offset"""
        self._lat_controller.set_offset(offset)


class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = deque(maxlen=10)

    def run_step(self, target_speed, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

            :param target_speed: target speed in Km/h
            :param debug: boolean for debugging
            :return: throttle control
        """
        current_speed = get_speed(self._vehicle)

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

            :param target_speed:  target speed in Km/h
            :param current_speed: current speed of the vehicle in Km/h
            :return: throttle/brake control
        """

        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class PIDLateralController():
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, offset=0, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param offset: distance to the center line. If might cause issues if the value
                is large enough to make the vehicle invade other lanes.
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._offset = offset
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoint):
        return self._pid_control(waypoint, self._vehicle.get_transform())

    def set_offset(self, offset):
        """Changes the offset"""
        self._offset = offset

    def getRightVector(rotation):
        cy = math.cos(math.radians(rotation.yaw))
        sy = math.sin(math.radians(rotation.yaw))
        cr = math.cos(math.radians(rotation.roll))
        sr = math.sin(math.radians(rotation.roll))
        cp = math.cos(math.radians(rotation.pitch))
        sp = math.sin(math.radians(rotation.pitch))

        x = cy * sp * sr - sy * cr
        y = sy * sp * sr + cy * cr
        z = -cp * sr
        return Vector3D(x, y, z)

    def _pid_control(self, waypoint, vehicle_transform):

        # Get the ego's location and forward vector
        ego_loc = vehicle_transform.location
        v_vec = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_vec.x, v_vec.y, 0.0])

        # Get the vector vehicle-target_wp
        if self._offset != 0:
            # Displace the wp to the side
            w_tran = waypoint
            r_vec = self.getRightVector(waypoint.rotation)
            w_loc = w_tran.location + carla.Location(x=self._offset*r_vec.x,
                                                     y=self._offset*r_vec.y)
        else:
            w_loc = waypoint.location

        w_vec = np.array([w_loc.x - ego_loc.x,
                          w_loc.y - ego_loc.y,
                          0.0])

        wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(v_vec)
        if wv_linalg == 0:
            _dot = 1
        else:
            _dot = math.acos(
                np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class MPCLateralController:
    def __init__(self, vehicle, N=10, dt=0.1, Q=np.diag([1, 1, 0.1, 0.1]), R=np.diag([1])):
        self._vehicle = vehicle
        self._N = N  # Number of time steps to predict
        self._dt = dt  # Time step size
        self._Q = Q  # State cost matrix
        self._R = R  # Input cost matrix
        self._last_steering = 0  # Store the last steering command

    def run_step(self, target_path):
        # Get the current vehicle state
        current_state = self._get_current_state()

        # Solve the MPC optimization problem
        steering = self._solve_mpc(current_state, target_path)

        # Apply steering command to the vehicle
        control = carla.VehicleControl()
        control.steer = float(steering)
        control.hand_brake = False
        control.manual_gear_shift = False
        self._last_steering = steering

        return control

    def _get_current_state(self):
        # Get the current vehicle state (position, orientation, speed)
        transform = self._vehicle.get_transform()
        velocity = self._vehicle.get_velocity()
        speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return np.array([transform.location.x, transform.location.y, transform.rotation.yaw, speed])

    def _solve_mpc(self, current_state, target_path):
        n_states = 4  # Number of states (x, y, yaw, speed)
        n_inputs = 1  # Number of inputs (steering)
        A, B = self._get_model_matrices()

        # Objective function
        P = cvxopt.matrix(np.block([
            [np.kron(np.eye(self._N), self._Q), np.zeros(
                (n_states*self._N, n_inputs*self._N))],
            [np.zeros((n_inputs*self._N, n_states*self._N)),
             np.kron(np.eye(self._N), self._R)]
        ]))
        q = cvxopt.matrix(np.zeros((n_states*self._N + n_inputs*self._N, 1)))

        # Equality constraints
        A_eq = self._build_aeq(A, B, n_states, n_inputs)
        b_eq = np.zeros((n_states * self._N, 1))
        b_eq[:n_states] = current_state - \
            np.hstack([target_path[0], [0, 0]]) if len(
                target_path) > 0 else current_state

        # Inequality constraints for path following
        G, h = self._build_inequalities(n_states, n_inputs, target_path)

        # Solve QP
        sol = cvxopt.solvers.qp(P, q, G, h, A_eq, cvxopt.matrix(b_eq))
        x = np.array(sol['x'])

        # Extract the first input (steering command)
        steering = x[n_states*self._N: n_states*self._N + n_inputs][0]

        return steering

    def _get_model_matrices(self):
        A = np.array([[1, 0, self._dt * np.cos(self._last_steering), 0],
                      [0, 1, self._dt * np.sin(self._last_steering), 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        B = np.array([[0],
                      [0],
                      [self._dt],
                      [0]])
        return A, B

    def _build_aeq(self, A, B, n_states, n_inputs):
        A_eq = np.zeros((n_states*self._N, n_states + n_inputs*self._N))
        for i in range(self._N):
            A_eq[n_states*i:n_states*(i+1), n_states*i:n_states*(i+1)] = A
            if i < self._N - 1:
                A_eq[n_states*i:n_states *
                     (i+1), n_states*(i+1):n_states*(i+2)] = -np.eye(n_states)
            if i < self._N:
                A_eq[n_states*i:n_states*(i+1), n_states*self._N +
                     n_inputs*i:n_states*self._N + n_inputs*(i+1)] = B
        return cvxopt.matrix(A_eq)

    def _build_inequalities(self, n_states, n_inputs, target_path):
        # This can be adapted based on the specific constraints you want to enforce
        # Placeholder, replace with actual inequality constraints
        G = cvxopt.matrix(0, (0, n_states*self._N + n_inputs*self._N), 'd')
        # Placeholder, replace with actual inequality constraints
        h = cvxopt.matrix(0, (0, 1), 'd')
        return G, h
