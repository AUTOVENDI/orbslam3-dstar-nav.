import numpy as np
import cv2
import yaml

# Load YAML config
with open("ros_map.yaml", "r") as f:
    config = yaml.safe_load(f)

img_path = config["image"]
resolution = config["resolution"]
origin = config["origin"]
occupied_thresh = config["occupied_thresh"]
free_thresh = config["free_thresh"]
negate = config["negate"]

# Load image as grayscale
image = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
if image is None:
    raise FileNotFoundError(f"Image not found: {img_path}")

# Normalize to 0.0–1.0
norm = image.astype(np.float32) / 255.0
if negate:
    norm = 1.0 - norm

# Create grid: start with all unknown (0)
grid = np.zeros_like(image, dtype=np.uint8)

# Assign values
grid[norm <= free_thresh] = 255     # Free space (white)
grid[norm >= occupied_thresh] = 77  # Obstacle (gray)

# Flip vertically to match ROS convention
grid = np.flipud(grid)

# Save to CSV
np.savetxt("initial_map.csv", grid, fmt='%d', delimiter=',')

print("✅ Saved grid to initial_map.csv")
print(f"ℹ️ Grid shape: {grid.shape}, resolution: {resolution}, origin: {origin}")

