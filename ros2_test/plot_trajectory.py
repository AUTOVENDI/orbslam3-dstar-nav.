import numpy as np
import matplotlib.pyplot as plt

# Load the trajectory file
data = np.loadtxt("KeyFrameTrajectory.txt")  # Make sure the file is in the same folder

# Extract x, y, z coordinates
x, y, z = data[:, 1], data[:, 2], data[:, 3]  # (index 0 is timestamp)

# Plot the trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label="Keyframe Trajectory", color="blue")
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_zlabel("Z Position")
ax.set_title("ORB-SLAM3 Keyframe Trajectory")
ax.legend()
plt.show()

