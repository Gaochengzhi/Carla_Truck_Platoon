import matplotlib.pyplot as plt
import numpy as np

def plot_location_relationships(csv_file_path):
    # Read the CSV data from the file
    data = np.genfromtxt(csv_file_path, delimiter=',')

    # Extract x, y, and yaw (ignoring z here)
    x = data[:, 0]
    y = data[:, 1]
    yaw = data[:, 3]

    # Convert yaw from degrees to radians for plotting
    yaw_rad = np.deg2rad(yaw)

    # Prepare the plot
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Location and Orientation')

    # Plot each position and orientation as an arrow
    for i, (xi, yi, yiaw) in enumerate(zip(x, y, yaw_rad)):
        dx = np.cos(yiaw)
        dy = np.sin(yiaw)
        ax.arrow(xi, yi, dx, dy, head_width=2, head_length=3, fc='k', ec='k')
        # Annotate with the index number
        ax.text(xi, yi, str(i), color="red", fontsize=12)

    plt.grid(True)
    plt.show()

# Call the function with your CSV file
plot_location_relationships('OpenDriveMap.csv')

