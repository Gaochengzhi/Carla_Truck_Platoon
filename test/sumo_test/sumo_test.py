import traci
import traci.constants as tc
import time
# Start SUMO in GUI mode
sumo_cmd = ["sumo-gui", "-c", "freeway.sumo.cfg"]
traci.start(sumo_cmd)

# Get the ID of the E0 edge
edge_id = "E0"

# Spawn 5 trucks on the E0 edge with an 8-meter distance between each truck
for i in range(5):
    truck_id = f"truck_{i}"
    traci.vehicle.add(truck_id, "t_0", typeID="truck", departPos=i*8+10)

# set time = 1000ms delay

# set view on the top of truck_0s
traci.gui.trackVehicle("View #0", "truck_0")
traci.gui.setZoom("View #0", 1000)

# Enable the top bird's eye view
traci.gui.setSchema("View #0", "real world")
# Simulate the scenario
while traci.simulation.getMinExpectedNumber() > 0:
    # Set the acceleration and speed of each truck
    for i in range(5):
        truck_id = f"truck_{i}"
        # traci.vehicle.setAccel(truck_id, 5.0)
        traci.vehicle.setSpeed(truck_id, 30)
    # Advance the simulation by one step
    traci.simulationStep()
    time.sleep(0.01)

# Close the TraCI connection
traci.close()