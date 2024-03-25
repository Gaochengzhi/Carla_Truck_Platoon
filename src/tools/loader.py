import random
import carla
from agent.base_vehicle import BaseVehicle



def load_agents(config):
    for i, agent_info in enumerate(config["agents"]):
        agent_info = config
        agent_info["start_point"] = 90
        agent_info["end_point"] = 18
        agent_info["name"] = "p_0"
        agent_info["port"] = 9999
        BaseVehicle(agent_info).start()




def load_conventional_agents(world, tm, config):
    try:
        spawn_point = world.get_map().get_spawn_points()
        config["spwan_list"] = random.sample(range(600, 700), 100)
        # config["spwan_list"] = [66] + \
        #     random.sample(range(68, 120), 47)
        # config["spwan_list"] = [750]
        # config["spwan_list"] = [136, 135, 173, 176, 295, 294, 153, 154]
        config["target_list"] = random.sample(range(100, 300), 100)
        # config["target_list"] = [1000]
        for i, spwan_target in enumerate(zip(config["spwan_list"], config["target_list"])):
            vehicle_bp = world.get_blueprint_library().filter(
                "vehicle.tesla.model3*")[0]
            vehicle_bp.set_attribute('role_name', f"agent_{i}")
            vehicle = world.spawn_actor(
                vehicle_bp, spawn_point[spwan_target[0]])
            tm.ignore_lights_percentage(vehicle, 100)
            # vehicle.set_simulate_physics(False)
            # vehicle.set_target_velocity(carla.Vector3D(0, 0, 105))
            vehicle.set_autopilot(True, tm.get_port())
            # tm.vehicle_percentage_speed_difference(vehicle, -50)
            # tm.vehicle_percentage_speed_difference(vehicle, 10)
    except Exception as e:
        print(f"load_conventional_agents error:{e}")
        pass
