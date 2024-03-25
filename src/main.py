from tools.world_manager import WorldManager
from tools.config_manager import config as cfg
from tools.input_manager import recieve_args
from agent.traffic_agent import TrafficFlowManager
from data.commuicate_manager import CommuniAgent
from data.recorder_manager import DataRecorder
from platoon.platoon_manager import Platoon
from util import destroy_all_actors, time_const, log_time_cost, clean_up, handle_exception
import time
import logging
import os
from tools.loader import load_agents, load_conventional_agents


def main():
    args = recieve_args()
    config = cfg.merge(args)
    world_manager = WorldManager(config)
    world = world_manager.get_world()
    traffic_manager = world_manager.get_traffic_manager()
    destroy_all_actors(world)
    main_com = MainCommuicator(config)
    # load_agents(config)
    # # load_platoon_agents(config)
    # load_conventional_agents(world, traffic_manager, config)
    plt = Platoon(config)

    TrafficFlowManager().start()
    # DataRecorder(config).start()
    data_recorder = DataRecorder(config)

    @time_const(fps=config["fps"])
    # @log_time_cost
    def run_step(world):
        data_recorder.run_step(world)
        world.tick()
    try:
        while True:
            run_step(world)
    except Exception as e:
        handle_exception(e)

    finally:
        main_com.send_obj("end")
        clean_up(world)
        destroy_all_actors(world)
        traffic_manager.set_synchronous_mode(False)
        time.sleep(1)
        main_com.close()
        logging.info("Simulation ended\n")


def MainCommuicator(config):
    world_control_center = CommuniAgent("World")
    world_control_center.init_publisher(config["main_port"])
    return world_control_center


if __name__ == "__main__":
    main()
