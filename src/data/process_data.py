import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft
from scipy.stats import describe
import seaborn as sns
from matplotlib.colors import PowerNorm
# Load the data
df = pd.read_csv('dataExample.csv')

# Calculate total speed and total acceleration
df['Speed'] = np.sqrt(df['velocity_x']**2 + df['velocity_y']**2 + df['velocity_z']**2)
df['ACC'] = np.sqrt(df['acceleration_x']**2 + df['acceleration_y']**2 + df['acceleration_z']**2)
speed_array = df['Speed'].to_numpy()
accel_array = df['ACC'].to_numpy()
fft_speed = 2.0 / len(speed_array) * np.abs(fft(speed_array)[:len(speed_array)//2])
fft_accel = 2.0 / len(accel_array) * np.abs(fft(accel_array)[:len(accel_array)//2])
# Frequency Domain Analysis for Speed

def plot_frequency_analysis(df):
    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    ymin_speed = np.percentile(fft_speed, 0)
    ymax_speed = np.percentile(fft_speed, 99.95)
    ymin_accel = np.percentile(fft_accel, 0) 
    ymax_accel = np.percentile(fft_accel, 99.95)


    ax1.plot(fft_speed, '-r', alpha=0.9)
    ax1.set_ylim(ymin_speed, ymax_speed) 
    ax2.plot(fft_accel, '-b', alpha=0.3)
    ax2.set_ylim(ymin_accel, ymax_accel)

    ax1.set_xlabel('Frequency')
    ax1.set_ylabel('Amplitude (Speed)', color='r')
    ax2.set_ylabel('Amplitude (Accel)', color='b')
    
    mean_speed = np.mean(fft_speed)
    std_speed = np.std(fft_speed)
    mean_accel = np.mean(fft_accel) 
    std_accel = np.std(fft_accel)
    
    ax1.text(0.05, 0.90, f"Speed Mean Amplitude: {mean_speed:.2f}\nSpeed Amplitude Std Dev: {std_speed:.2f}", 
         transform=ax1.transAxes, color='r')

    ax1.text(0.05, 0.70, f"Accel Mean Amplitude: {mean_accel:.2f}\nAccel Amplitude Std Dev: {std_accel:.2f}",
         transform=ax1.transAxes, color='b') 

    plt.tight_layout()
    plt.savefig('frequency_analysis.png')

# Calculate Angular Velocitys
df['Angular_Velocity'] = np.sqrt(df['angular_velocity_x']**2 + df['angular_velocity_y']**2 + df['angular_velocity_z']**2)
angular_velocity_array = df['Angular_Velocity'].to_numpy()

# Calculating Mean and Standard Deviation
velocity_mean, velocity_std = describe(speed_array)[2], np.sqrt(describe(speed_array)[3])
acceleration_mean, acceleration_std = describe(accel_array)[2], np.sqrt(describe(accel_array)[3])
angular_velocity_mean, angular_velocity_std = describe(angular_velocity_array)[2], np.sqrt(describe(angular_velocity_array)[3])

# Flow Rate, Queue Length, and Duration
df['Queue'] = df['Speed'] < 2
queue_length = df.groupby('time')['Queue'].sum()  # Sum of vehicles with speed < 2 at each time
flow_rate = df.groupby('time')['vehicle_id'].nunique()  # Unique vehicle count per time unit
duration = df[df['Queue']].groupby('vehicle_id')['time'].count()  # Count of time units each vehicle spent in queue

# Printing the results
print(f"Velocity: Mean = {velocity_mean}, Std = {velocity_std}")
print(f"Acceleration: Mean = {acceleration_mean}, Std = {acceleration_std}")
print(f"Angular Velocity: Mean = {angular_velocity_mean}, Std = {angular_velocity_std}")
print(f"Average Queue Length: {queue_length.mean()}")
print(f"Average Flow Rate: {flow_rate.mean()}")
print(f"Average Duration in Queue: {duration.mean()}")



def single_trajectories(df):
    vehicle_ids = df['vehicle_id'].unique()
    vehicle_subset = np.random.choice(vehicle_ids, 20, replace=False)
    df = df[df['vehicle_id'].isin(vehicle_subset)]

# Initialize figures 
    fig, axs = plt.subplots(20, 4, figsize=(16, 48))

# Loop through vehicles  
    for i, vid in enumerate(vehicle_subset):
  # Filter to current vehicle 
      df_vid = df[df['vehicle_id'] == vid]
  
#   Plot timeseries  
      axs[i,0].plot(df_vid['time'], df_vid['Speed'])
      if i == 0:
        axs[i,0].set_title(f'Vehicle Velocity')
        axs[i,1].set_title(f'Vehicle Acceleration')
        axs[i,2].set_title(f'Vehicle Angular Velocity Z')
        axs[i,3].set_title(f'Vehicle Control Steer')
  
      axs[i,1].plot(df_vid['time'], df_vid['ACC'])
   
      axs[i,2].plot(df_vid['time'], df_vid['angular_velocity_z'])
  
      axs[i,3].plot(df_vid['time'], df_vid['control_steer'])
#   Save figure
    fig.savefig(f'vehicle_{vid}_plots.png')
    return df_vid

df_vid = single_trajectories(df)

# Aggregate statistics
vel_mean = df_vid['control_steer'].mean()
vel_std = df_vid['control_steer'].std()

acc_mean = df_vid['control_throttle'].abs().mean()
acc_std = df_vid['control_throttle'].std()

ang_vel_mean = df_vid['control_brake'].abs().mean()
ang_vel_std = df_vid['control_brake'].std()

max_acc_mag = (df_vid[['acceleration_x', 'acceleration_y', 'acceleration_z']]**2).sum(axis=1).max()  

hard_brake_count = (df_vid['control_brake'] > 0.6).sum()

jerk = df_vid['acceleration_x'].diff().abs()
max_jerk = jerk.max()

print(f'Velocity Mean: {vel_mean:.3f}, Std Dev: {vel_std:.3f}') 
print(f'Acceleration Mean: {acc_mean:.3f}, Std Dev: {acc_std:.3f}')
print(f'Angular Velocity Mean: {ang_vel_mean:.3f}, Std Dev: {ang_vel_std:.3f}')
print(f'Max Acceleration Magnitude: {max_acc_mag:.3f}')
print(f'Hard Braking Events: {hard_brake_count}') 
print(f'Max Jerk: {max_jerk:.3f}')
