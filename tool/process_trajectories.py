import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import shutil
import os

# Configuration
platoon_length = 5
plt_members = ["p_" + str(i) for i in range(platoon_length)]
data_path = "../data/data.csv"
res_dir = "../res/"
time_step_range = [0, 3300]  # Assuming this is in terms of row indices for simplicity
time_headway = 0.5  # seconds
vehicle_lenghth = 19.55368
mini_space = 2

# Ensure the res directory exists
os.makedirs(res_dir, exist_ok=True)

# Load the data
df = pd.read_csv(data_path)

# Filter data based on the time_step_range
# df_filtered = df[(df['time'] >= time_step_range[0]) & (df['time'] <= time_step_range[1]) & time % 5 ==0
df_filtered = df[(df['time'] >= time_step_range[0]) & (df['time'] <= time_step_range[1]) & (df['time'] % 3 == 0)]

# Helper function to calculate distance between two points in XY plane
def calc_distance(x1, y1, x2, y2):
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Initialize plots
fig, axs = plt.subplots(3, 1, figsize=(12, 12))

# Step-Gap Plot
for i in range(platoon_length - 1):
    gap_errors = []
    for _, grp in df_filtered.groupby('time'):
        if f'p_{i}' in grp['vehicle_id'].values and f'p_{i+1}' in grp['vehicle_id'].values:
            p1 = grp[grp['vehicle_id'] == f'p_{i}']
            p2 = grp[grp['vehicle_id'] == f'p_{i+1}']
            gap = calc_distance(p1['location_x'].values[0], p1['location_y'].values[0],
                                # p2['location_x'].values[0], p2['location_y'].values[0])-vehicle_lenghth
                                p2['location_x'].values[0], p2['location_y'].values[0])-vehicle_lenghth- p2['velocity_x'].values[0]*time_headway - mini_space
            gap_errors.append(gap)
    axs[0].plot(gap_errors, label=f'p_{i} to p_{i+1}')
# set_limit y -8, 8
axs[0].set_ylim(-8, 30)
axs[0].set_title('Step-Gap Plot')
axs[0].set_xlabel('Step')
axs[0].set_ylabel('Gap error (m)')

# Speed-Step Plot
for i in range(platoon_length):
    speeds = df_filtered[df_filtered['vehicle_id'] == f'p_{i}']['velocity_x'].values
    axs[1].plot(speeds, label=f'p_{i}')
axs[1].set_ylim(0, 32)
axs[1].set_title('Speed-Step Plot')
axs[1].set_xlabel('Step')
axs[1].set_ylabel('Speed (m/s)')

# Acc-Step Plot
for i in range(platoon_length):
    accs = df_filtered[df_filtered['vehicle_id'] == f'p_{i}']['acceleration_x'].values
    axs[2].plot(accs, label=f'p_{i}')
axs[2].set_ylim(-4, 2)
axs[2].set_title('Acceleration-Step Plot')
axs[2].set_xlabel('Step')
axs[2].set_ylabel('Acceleration (m/s^2)')

# show grids
for i in range(3):
    axs[i].grid()
    axs[i].legend()

# Heatmap Plot
# distances = []
# speeds = []
# for _, grp in df_filtered.groupby('time'):
#     for i in range(platoon_length):
#         if f'p_{i}' in grp['vehicle_id'].values:
#             p = grp[grp['vehicle_id'] == f'p_{i}']
#             start_x, start_y = df[df['vehicle_id'] == f'p_{i}'].iloc[0][['location_x', 'location_y']]
#             distance = calc_distance(start_x, start_y, p['location_x'].values[0], p['location_y'].values[0])
#             speed = np.sqrt(p['velocity_x'].values[0] ** 2 + p['velocity_y'].values[0] ** 2)
#             distances.append(distance)
#             speeds.append(speed)
# axs[3].scatter(range(len(distances)), distances, c=speeds, cmap='hot')
# axs[3].set_title('Distance-Speed Heatmap')
# axs[3].set_xlabel('Step')
# axs[3].set_ylabel('Distance from Start (m)')
# axs[3].colorbar(label='Speed (m/s)')

plt.tight_layout()

# Save figures
# for i, ax in enumerate(axs, start=1):
fig.savefig(f"{res_dir}plot_{1}.png")

# Copy data.csv to a timestamped file in res directory
timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
shutil.copy2(data_path, f"{res_dir}{timestamp}caccsamtrajectoriesloss.csv")
print("Analysis completed and files saved.")
