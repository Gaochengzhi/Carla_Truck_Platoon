from config.config_manager import config as cfg
from config.input_manager import config_args
from world.world_manager import WorldManager
from platoon.platoon_manager import Platoon
import logging
import time
from util import destroy_all_actors

def main():
    args = config_args()
    config = cfg.merge(args)

    carla_world = WorldManager(config)
    world, client, filtered_waypoints, traffic_manager = get_world_instence(carla_world)
    plt = Platoon(world, client,filtered_waypoints,config)
    try:
        while True:
            world.tick()
            time.sleep(0.01)
    except KeyboardInterrupt:
        destroy_all_actors(world)

    pass



def get_world_instence(scenario_loader):
    map = scenario_loader.get_map()
    world = scenario_loader.get_world()
    client = scenario_loader.get_client()
    filtered_waypoints = scenario_loader.get_filtered_waypoints()
    traffic_manager = scenario_loader.get_trafficmanager()
    world.tick()
    return world, client, filtered_waypoints, traffic_manager


if __name__ == "__main__":
    main()