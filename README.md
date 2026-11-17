# No More Offline Maps: Real-Time D* Lite Planning with Visual SLAM

Code accompanying the paper *"No More Offline Maps: Real-Time D-star Planning with Visual SLAM"*
(Ahyan Ahmed, Maha Masood, Haider Ali — NUST, CEME).

The system has two phases:

1. **Mapping** – ORB-SLAM3 builds a map of the environment while the robot is driven around manually.
2. **Navigation** – the map is converted into a 2D occupancy grid and used by a D* Lite planner to drive the robot to a goal.

---

## Repository Structure

```
orbslam3-dstar-nav/
├── ros2_test/                 # ROS 2 workspace for ORB-SLAM3 (mapping)
│   ├── src/ros2_orb_slam3/    # ORB-SLAM3 ROS 2 package
│   └── save_as_occupancy.py   # Point cloud (.ply) -> 2D map (.pgm + .yaml)
├── gridmaking/                # Map image (.pgm) -> occupancy grid (.csv)
└── dstar_lite_pathplanner/    # D* Lite planner + robot navigation (main.py)
```

---

## Requirements

**System**
- Ubuntu 22.04
- ROS 2 Humble
- C++17, CMake ≥ 3.8

**ORB-SLAM3 dependencies**
- OpenCV ≥ 4.2
- Eigen3 ≥ 3.3
- Pangolin
- PCL (`pcl_conversions`, `pcl_msgs`)
- ROS 2 packages: `cv_bridge`, `image_transport`, `sensor_msgs`, `nav_msgs`, `tf2_geometry_msgs`
- DBoW2, g2o and Sophus (bundled in `Thirdparty/`)

**Python packages**
```bash
pip install numpy pandas opencv-python pyyaml pygame open3d pillow matplotlib
```

**Hardware (real-world tests)**
- Kinect RGB-D camera
- Mobile robot publishing `/odom` and accepting `/cmd_vel`

---

## Build

```bash
cd ros2_test
colcon build --symlink-install
source install/setup.bash
```

> Unzip `orb_slam3/Vocabulary.zip` first if `ORBvoc.txt.bin` is missing.

---

## How to Run

### 1. Build the map with ORB-SLAM3

Start the camera, then run ORB-SLAM3 and drive the robot around the environment:

```bash
ros2 run ros2_orb_slam3 mono_node_cpp \
  /home/autovendi/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin \
  /home/autovendi/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular/TUM1.yaml
```

When you stop the node, the reconstructed map (point cloud / keyframes) is saved.

*(Optional)* Convert the point cloud into a 2D map image:

```bash
python3 save_as_occupancy.py <map.ply> [resolution] [map_name]
```

### 2. Convert the map into a grid

Use the scripts in `gridmaking/` to turn the `.pgm` map into a `.csv` occupancy grid:

```bash
cd gridmaking
python3 conversion.py        # or convert_pgm_to_csv.py
```

Set the input/output file names at the top of the script as needed.

### 3. Set up the planner

Copy the generated `.csv` grid into `dstar_lite_pathplanner/dstar_lite_pathplanner/`
(or any location), then update the map path in `main.py`:

```python
csv_grid = pd.read_csv("initial_map.csv", header=None)  # <- path to your grid
```

Start and goal cells can also be adjusted in `main.py`.

### 4. Run D* Lite navigation

```bash
cd dstar_lite_pathplanner/dstar_lite_pathplanner
python3 main.py
```

The planner waits for a goal on the `/navigation_goal` topic (sent from the mobile app),
plans a path, and shows it in a window. Press **Space** to start moving the robot.
Obstacles detected by the Kinect make the robot stop and resume once the path is clear.

---

## Acknowledgements

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) and its [ROS 2 wrapper](https://github.com/Mechazo11/ros2_orb_slam3)
- [D* Lite path planner](https://github.com/Sollimann/Dstar-lite-pathplanner)

This research is part of NUST Flagship funding FSP-23-13.
