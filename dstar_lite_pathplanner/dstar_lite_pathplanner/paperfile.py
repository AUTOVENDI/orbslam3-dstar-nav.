import numpy as np
import pandas as pd
import time
import rclpy
import math
import threading
from gui import Animation
from d_star_lite import DStarLite
from grid import SLAM
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.task import Future
from rclpy.node import Node
from grid import OBSTACLE

OBSTACLE = 255
FRONT_OFFSET = 0.0


# =======================
# Utils
# =======================
def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def fetch_odometry_once(node, timeout=2.0):
    future = Future()

    def cb(msg):
        future.set_result(msg)

    sub = node.create_subscription(Odometry, '/odom', cb, 10)
    start = time.time()

    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > timeout:
            break

    node.destroy_subscription(sub)
    return future.result() if future.done() else None


# =======================
# Controller
# =======================
class CmdVelController(Node):
    def __init__(self, resolution=0.5):
        super().__init__('cmd_vel_controller')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.position = None
        self.yaw = None
        self.yaw_filt = None

        # ---- FILTER PARAMS (CRITICAL) ----
        self.YAW_ALPHA = 0.85   # closer to 1 = more smoothing

        self.resolution = resolution
        self.start_grid = (12, 45)

        self.path = []
        self.path_index = 0
        self.following = False

        # ---- HYSTERESIS STATE ----
        self.heading_locked = False

    def odom_cb(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        raw_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        # ---- LOW PASS FILTER ----
        if self.yaw_filt is None:
            self.yaw_filt = raw_yaw
        else:
            self.yaw_filt = (
                self.YAW_ALPHA * self.yaw_filt
                + (1.0 - self.YAW_ALPHA) * raw_yaw
            )

        self.yaw = self.yaw_filt

    def world_to_grid(self, pos):
        gx = round((pos[0] / self.resolution) + self.start_grid[0])
        gy = round((pos[1] / self.resolution) + self.start_grid[1])
        return gx, gy

    def grid_to_world(self, grid):
        gx, gy = grid
        wx = (gx - self.start_grid[0]) * self.resolution
        wy = (gy - self.start_grid[1]) * self.resolution
        return wx, wy

    def update_path(self, path):
        self.path = path
        self.path_index = 0
        self.following = False
        self.heading_locked = False

    def start_following(self):
        self.following = True

    def stop(self):
        self.publisher.publish(Twist())

    # =======================
    # STABLE MOTION CONTROL
    # =======================
    def move_along_path(self):
        if not self.following or not self.path or self.position is None:
            return

        if self.path_index + 1 >= len(self.path):
            self.stop()
            self.following = False
            return

        # ---- WORLD SPACE LOOKAHEAD ----
        target_grid = self.path[self.path_index + 1]
        target_world = self.grid_to_world(target_grid)

        dx = target_world[0] - self.position[0]
        dy = target_world[1] - self.position[1]

        distance = math.hypot(dx, dy)
        if distance < 0.25:
            self.path_index += 1
            self.heading_locked = False
            return

        desired_yaw = math.atan2(dy, dx)

        yaw = self.yaw + FRONT_OFFSET
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        angle_diff = desired_yaw - yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # ---- CONTROL PARAMETERS ----
        MAX_LIN = 0.30
        MAX_ANG = 0.7

        LOCK_ANGLE = 0.18      # enter straight mode
        UNLOCK_ANGLE = 0.30    # exit straight mode

        twist = Twist()

        # ---- HEADING HYSTERESIS (THIS KILLS WOBBLE) ----
        if self.heading_locked:
            if abs(angle_diff) > UNLOCK_ANGLE:
                self.heading_locked = False
        else:
            if abs(angle_diff) < LOCK_ANGLE:
                self.heading_locked = True

        if self.heading_locked:
            twist.angular.z = 0.0
            twist.linear.x = -MAX_LIN
        else:
            twist.angular.z = max(min(MAX_ANG * angle_diff, MAX_ANG), -MAX_ANG)
            twist.linear.x = -0.08  # creep while aligning

        self.publisher.publish(twist)


# =======================
# Planner helper
# =======================
def compute_axis_aligned_path(start, goal, grid_map):
    path = []
    x0, y0 = start
    xg, yg = goal

    step = 1 if xg > x0 else -1
    for x in range(x0, xg + step, step):
        if grid_map.occupancy_grid_map[x, y0] == OBSTACLE:
            return None
        path.append((x, y0))

    step = 1 if yg > y0 else -1
    for y in range(y0 + step, yg + step, step):
        if grid_map.occupancy_grid_map[xg, y] == OBSTACLE:
            return None
        path.append((xg, y))

    return path


# =======================
# Main
# =======================
def main():
    rclpy.init()

    grid = pd.read_csv("initial_map.csv", header=None).values.astype(np.uint8)
    grid[grid == 1] = OBSTACLE

    start = (12, 45)
    goal = (3, 53)

    gui = Animation("Hybrid Planner", 10, 10, 0,
                    grid.shape[0], grid.shape[1],
                    start, goal, 4)

    gui.world.set_map(grid)
    slam = SLAM(gui.world, 4)

    follower = CmdVelController()
    msg = fetch_odometry_once(follower)
    if msg:
        follower.odom_cb(msg)

    executor = MultiThreadedExecutor()
    executor.add_node(slam)
    executor.add_node(follower)
    threading.Thread(target=executor.spin, daemon=True).start()

    while follower.position is None:
        rclpy.spin_once(follower, timeout_sec=0.1)

    start_grid = follower.world_to_grid(follower.position)
    gui.set_start(start_grid)
    gui.set_position(start_grid)

    dstar = DStarLite(gui.world, start_grid, goal)
    _, slam_map = slam.rescan(start_grid)

    path = compute_axis_aligned_path(start_grid, goal, slam_map)
    if path is None:
        path, _, _ = dstar.move_and_replan(start_grid)

    follower.update_path(path)

    while not gui.done:
        if follower.position:
            new_pos = follower.world_to_grid(follower.position)
            gui.set_position(new_pos)

            if new_pos != start_grid:
                path = compute_axis_aligned_path(new_pos, goal, slam_map)
                if path is None:
                    dstar.s_start = new_pos
                    _, slam_map = slam.rescan(new_pos)
                    path, _, _ = dstar.move_and_replan(new_pos)

                follower.update_path(path)
                start_grid = new_pos

        gui.run_game(path)
        if gui.space_pressed:
            follower.start_following()

        follower.move_along_path()

    follower.stop()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
