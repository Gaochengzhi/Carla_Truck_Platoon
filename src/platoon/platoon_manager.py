from agent.truck_vehicle_agent import TruckVehicleAgent


class Platoon():
    def __init__(self, config):
        self.config = config
        self.spawn_platoon_agents(config)

    def spawn_platoon_agents(self, config):
        plt_member_config = config
        for i, start_end in enumerate(zip(config["spawn_list"], config["target_list"])):
            plt_member_config["name"] = f"p_{i}"
            plt_member_config["topology"] = {
                "LV": 0 if i != 0 else -1,
                "FV": i-1 if i-1 >= 0 else -1,
                "RV": i+1 if i+1 < len(config["spawn_list"]) else -1
            }
            plt_member_config["port"] = int(config["base_port"]+i)
            plt_member_config["start_point"] = start_end[0]
            plt_member_config["end_point"] = start_end[1]
            TruckVehicleAgent(plt_member_config).start()
