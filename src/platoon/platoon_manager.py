from agent.truck_vehicle_agent import TruckVehicleAgent


class Platoon():
    def __init__(self, config):
        self.config = config
        self.spawn_platoon_agents(config)
        # self.

    def spawn_platoon_agents(self, config):
        plt_member_config = {}
        for i, spawn_target in enumerate(zip(config["spawn_list"], config["target_list"])):
            plt_member_config["name"] = f"p_{i}"
            plt_member_config["topology"] = {
                "LV": 0 if i != 0 else -1,
                "FV": i-1 if i-1 >= 0 else -1,
                "RV": i+1 if i+1 < len(config["spawn_list"]) else -1
            }
            plt_member_config["fps"] = config["fps"]
            plt_member_config["base_port"] = 9985
            plt_member_config["port"] = int(9985+i)
            plt_member_config["start_point"] = spawn_target[0]
            plt_member_config["end_point"] = spawn_target[1]
            plt_member_config["traffic_agent_port"] = config["traffic_agent_port"]
            plt_member_config["main_port"] = config["main_port"]
            plt_member_config["ignore_traffic_light"] = False
            TruckVehicleAgent(plt_member_config).start()
