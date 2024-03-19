import random
import math
import os
import logging
import carla
from prediction.predict_baseline import predict
from view.debug_manager import draw_future_locations
from util import connect_to_server, time_const, log_time_cost, batch_process_vehicles, get_speed, handle_exception
from cythoncode.cutil import is_within_distance_obs
from agent.base_agent import BaseAgent
import time
from tools.config_manager import config as cfg
from pyinstrument import Profiler
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
import numpy as np


def is_within_angle_range(ego_location, target_location, ego_yaw, angle_interval):
    dx = target_location.x - ego_location[0]
    dy = target_location.y - ego_location[1]
    angle = np.degrees(np.arctan2(dy, dx))
    angle_diff = (angle - ego_yaw + 180) % 360 - 180
    return angle_interval[0] <= angle_diff <= angle_interval[1]


class TrafficFlowManager(BaseAgent):
    def __init__(
        self,
    ) -> None:
        self.config = cfg.config
        BaseAgent.__init__(self, "TrafficFlow",
                           self.config["traffic_agent_port"])
        # self.profiler = Profiler(interval=0.001)

    def run(self):
        @time_const(fps=self.config["fps"])
        # @log_time_cost(name="traffic")
        def run_step(world):
            # self.profiler.start()
            try:
                perception_res = batch_process_vehicles(
                    world, predict, self.config["fps"])
                self.communi_agent.send_obj(perception_res)
            except Exception as e:
                handle_exception(e)

            # self.profiler.stop()
            # self.profiler.print(show_all=True)
        client, world = connect_to_server(
            self.config["carla_timeout"], self.config["carla_port"])
        self.init_base_agent()
        # time.sleep(1)
        while True:
            run_step(world)

