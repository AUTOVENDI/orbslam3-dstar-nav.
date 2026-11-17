import numpy as np
import pandas as pd
import time
import rclpy
import math
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from gui import Animation
from d_star_lite import DStarLite
from grid import SLAM, OBSTACLE

OBSTACLE = 255
FRONT_OFFSET = 0.0


# =======================
# Utils
# =======================
def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def fetch_odometry_once(node, timeout=2.0):
    future = Future()

    def cb(msg):
        future.set_result(msg)

    sub = node.create_subscription(Odometry, "/odom", cb, 10)
    start = time.time()

    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > timeout:
            break

    node.destroy_subscription(sub)
    return future.result() if future.done() else None


# =======================
# Controller (PATH2 MOVEMENT)
# =======================
class CmdVelController(Node):
    def __init__(self, resolution, grid_shape):
        super().__init__("cmd_vel_controller")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        self.position = None
        self.yaw = None

        self.path = []
        self.path_index = 0
        self.following = False

        self.resolution = resolution
        self.map_w = grid_shape[0]
        self.map_h = grid_shape[1]

        # GUI-centered origin (path3 style)
        self.origin_x = self.map_w // 2
        self.origin_y = self.map_h // 2

    def odom_cb(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    # =======================
    # WORLD <-> GRID (GUI FRAME)
    # =======================
    def world_to_grid(self, pos):
        gx = round(pos[0] / self.resolution + self.origin_x)
        gy = round(pos[1] / self.resolution + self.origin_y)
        return gx, gy

    def grid_to_world(self, grid):
        return (
            (grid[0] - self.origin_x) * self.resolution,
            (grid[1] - self.origin_y) * self.resolution
        )

    def update_path(self, path):
        self.path = path
        self.path_index = 0
        self.following = False

    def start_following(self):
        self.following = True

    def stop(self):
        self.pub.publish(Twist())

    # =======================
    # EXACT PATH2 MOVEMENT
    # =======================
    def move_along_path(self):
        if not self.following or not self.path or self.position is None:
            return

        if self.path_index >= len(self.path) - 1:
            self.stop()
            self.following = False
            return

        target_grid = self.path[self.path_index + 1]
        target_world = self.grid_to_world(target_grid)

        dx = target_world[0] - self.position[0]
        dy = target_world[1] - self.position[1]

        if math.hypot(dx, dy) < 0.25:
            self.path_index += 1
            return

        # ---- PATH2 LOGIC (DO NOT TOUCH) ----
        desired_yaw = math.atan2(dy, dx) + math.pi
        desired_yaw = math.atan2(math.sin(desired_yaw), math.cos(desired_yaw))

        yaw = self.yaw + FRONT_OFFSET
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        angle_err = desired_yaw - yaw
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

        MAX_LIN = 0.30
        MAX_ANG = 0.7
        ALIGN_ANGLE = 0.9

        cmd = Twist()

        if abs(angle_err) > ALIGN_ANGLE:
            cmd.linear.x = 0.0
            cmd.angular.z = max(min(0.7 * angle_err, MAX_ANG), -MAX_ANG)
        else:
            cmd.linear.x = -MAX_LIN
            cmd.angular.z = 0.0

        # REQUIRED (PATH2)
        cmd.angular.z *= -1.0

        self.pub.publish(cmd)


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

    grid = pd.read_csv("map_grid2.csv", header=None).values.astype(np.uint8)
    grid[grid == 1] = OBSTACLE

    RESOLUTION = 0.2
    GOAL = (49, 40)

    gui = Animation(
        "Hybrid Planner",
        10, 10, 0,
        grid.shape[0], grid.shape[1],
        None, None, 4
    )

    gui.world.set_map(grid)
    slam = SLAM(gui.world, 4)

    follower = CmdVelController(RESOLUTION, grid.shape)

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
    gui.set_goal(GOAL)

    dstar = DStarLite(gui.world, start_grid, GOAL)
    _, slam_map = slam.rescan(start_grid)

    path = compute_axis_aligned_path(start_grid, GOAL, slam_map)
    if path is None:
        path, _, _ = dstar.move_and_replan(start_grid)

    follower.update_path(path)

    while not gui.done:
        if follower.position:
            new_pos = follower.world_to_grid(follower.position)
            gui.set_position(new_pos)

            if new_pos != start_grid:
                path_new = compute_axis_aligned_path(new_pos, GOAL, slam_map)
                if path_new is None:
                    dstar.s_start = new_pos
                    _, slam_map = slam.rescan(new_pos)
                    path_new, _, _ = dstar.move_and_replan(new_pos)

                if path_new != follower.path:
                    follower.update_path(path_new)

                start_grid = new_pos

        gui.run_game(follower.path)

        if gui.space_pressed:
            follower.start_following()

        follower.move_along_path()

    follower.stop()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

