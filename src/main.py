from tools.world_manager import WorldManager
from tools.config_manager import config as cfg
from tools.input_manager import recieve_args
from agent.traffic_agent import TrafficFlowManager
from data.commuicate_manager import CommuniAgent
from data.recorder_manager import DataRecorder
from platoon.platoon_manager import Platoon
from util import destroy_all_actors, time_const, log_time_cost
import time
import logging
from tools.loader import load_agents, load_platoon_agents, load_conventional_agents


def main():
    args = recieve_args()
    config = cfg.merge(args)
    world_manager = WorldManager(config)
    world = world_manager.get_world()
    TM = world_manager.get_traffic_manager()
    destroy_all_actors(world)
    main_com = MainCommuicator(config)
    main_com.send_obj("start")
    # load_platoon_agents(config)
    # load_conventional_agents(world, TM, config)
    plt = Platoon(config)

    TrafficFlowManager().start()
    DataRecorder(config).start()

    # @log_time_cost
    @time_const(fps=config["fps"])
    def run_step(world):
        world.tick()
    try:
        while True:
            run_step(world)
    except Exception as e:
        logging.error(f"main error:{e}")
    finally:
        main_com.send_obj("end")
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        TM.set_synchronous_mode(False)
        destroy_all_actors(world)
        time.sleep(1)
        main_com.close()
        logging.info("Simulation ended\n")


def MainCommuicator(config):
    world_control_center = CommuniAgent("World")
    world_control_center.init_publisher(config["main_port"])
    return world_control_center


if __name__ == "__main__":
    main()
