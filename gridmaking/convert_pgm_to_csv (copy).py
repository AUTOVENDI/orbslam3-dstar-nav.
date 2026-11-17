import numpy as np
import cv2
import csv

# ==========================
# CONFIG
# ==========================
PGM_PATH = "map.pgm"          # input pgm
CSV_PATH = "map_grid.csv"    # output csv

FREE_THRESH = 240     # >= this → free (white)
OCC_THRESH  = 50      # <= this → occupied (black)

# ==========================
# LOAD PGM IMAGE
# ==========================
img = cv2.imread(PGM_PATH, cv2.IMREAD_GRAYSCALE)

if img is None:
    raise RuntimeError("Failed to load PGM image")

height, width = img.shape
print(f"Loaded map: {width} x {height}")

# ==========================
# CONVERT TO GRID
# ==========================
grid = np.zeros((height, width), dtype=int)

for y in range(height):
    for x in range(width):
        pixel = img[y, x]

        if pixel >= FREE_THRESH:
            grid[y, x] = 0        # free
        elif pixel <= OCC_THRESH:
            grid[y, x] = 1        # obstacle
        else:
            grid[y, x] = -1       # unknown

# ==========================
# SAVE TO CSV
# ==========================
with open(CSV_PATH, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(grid)

print(f"CSV grid saved to: {CSV_PATH}")

