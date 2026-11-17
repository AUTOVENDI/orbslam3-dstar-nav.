import matplotlib.pyplot as plt
import numpy as np

# Function to read the KeyFrameTrajectory.txt file
def read_keyframe_trajectory(file_path):
    timestamps = []
    positions = []
    quaternions = []
    
    with open(file_path, 'r') as f:
        for line in f:
            data = line.strip().split()
            timestamps.append(float(data[0]))  # Timestamp
            positions.append([float(data[1]), float(data[2]), float(data[3])])  # Position (X, Y, Z)
            quaternions.append([float(data[4]), float(data[5]), float(data[6]), float(data[7])])  # Quaternion (W, X, Y, Z)
    
    return np.array(timestamps), np.array(positions), np.array(quaternions)

# Read the trajectory data
timestamps, positions, quaternions = read_keyframe_trajectory('KeyFrameTrajectory.txt')

# Create a 3D plot to visualize the camera trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the camera positions
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label="Camera Trajectory", color='b')

# Label the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set the title
ax.set_title('Camera Trajectory Visualization')

# Display the plot
plt.show()

