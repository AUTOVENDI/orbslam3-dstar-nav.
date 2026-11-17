import cv2
import numpy as np
import csv

# ==========================
# CONFIG (MATCH OLD MAP)
# ==========================
PGM_PATH = "house2.pgm"
CSV_PATH = "map_grid2.csv"

GRID_SIZE = 100   # 🔥 MUST match initial_map.csv

FREE_THRESH = 240
OCC_THRESH  = 50

# ==========================
# LOAD IMAGE
# ==========================
img = cv2.imread(PGM_PATH, cv2.IMREAD_GRAYSCALE)
if img is None:
    raise RuntimeError("Failed to load PGM")

# ==========================
# RESIZE TO 100x100
# ==========================
img_resized = cv2.resize(
    img,
    (GRID_SIZE, GRID_SIZE),
    interpolation=cv2.INTER_AREA
)

grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)

# ==========================
# THRESHOLDING
# ==========================
for y in range(GRID_SIZE):
    for x in range(GRID_SIZE):
        pixel = img_resized[y, x]

        if pixel >= FREE_THRESH:
            grid[y, x] = 0      # free
        else:
            grid[y, x] = 1      # obstacle (unknown = blocked)

# ==========================
# SAVE CSV
# ==========================
with open(CSV_PATH, "w", newline="") as f:
    csv.writer(f).writerows(grid)

print("Saved map_grid.csv with shape:", grid.shape)

