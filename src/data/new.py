import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Parse CSV into DataFrame
df = pd.read_csv('data.csv')  

# Traffic Density
num_vehicles = len(df['vehicle_id'].unique())  
x_range = df['location_x'].max() - df['location_x'].min()
y_range = df['location_y'].max() - df['location_y'].min()
area = x_range * y_range
density = num_vehicles / area  

# Traffic Efficiency
efficiency = df['velocity_x'].mean()

# Traffic Stability  
stability = df[['acceleration_x', 'acceleration_y']].var().mean()  

# Plots
fig, axs = plt.subplots(1, 3, figsize=(15, 5))

axs[0].set_title('Traffic Density')
axs[0].bar([0], [density])

axs[1].set_title('Traffic Efficiency')
axs[1].plot(efficiency)

axs[2].set_title('Traffic Stability')
axs[2].plot(stability)

for ax in axs:
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    
fig.tight_layout()

fig.savefig('traffic_metrics.png')

print(f'Density: {density:.2f}') 
print(f'Efficiency: {efficiency:.2f}')
print(f'Stability: {stability:.2f}')
