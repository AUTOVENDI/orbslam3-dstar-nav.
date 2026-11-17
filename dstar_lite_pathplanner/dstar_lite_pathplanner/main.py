import numpy as np
import pandas as pd
import time
import rclpy
import math
import os
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM

# ---------------- CONSTANTS ----------------
OBSTACLE = 255
UNOCCUPIED = 0
FRONT_OFFSET = math.radians(0.0)

# ---------------- UTILITIES ----------------
def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def fetch_odometry_once(node: Node, timeout_sec=2.0):
    future = Future()

    def cb(msg):
        future.set_result(msg)

    sub = node.create_subscription(Odometry, '/odom', cb, 10)
    start = time.time()

    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > timeout_sec:
            break

    node.destroy_subscription(sub)
    return future.result() if future.done() else None

# ---------------- GOAL LISTENER ----------------
class GoalListener(Node):
    def __init__(self):
        super().__init__('goal_listener')
        self.goal_name = None
        self.create_subscription(
            String,
            '/navigation_goal',
            self.callback,
            10
        )

    def callback(self, msg):
        self.goal_name = msg.data
        self.get_logger().info(f"📨 Goal received: {self.goal_name}")

# ---------------- CMD VEL CONTROLLER ----------------
class CmdVelController(Node):
    def __init__(self, resolution=0.5):
        super().__init__('cmd_vel_controller')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.position = None
        self.orientation = None
        self.resolution = resolution
        self.start_grid = (3, 45)

        self.path = []
        self.path_index = 0
        self.following = False

    def odom_callback(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.orientation = msg.pose.pose.orientation.z

    def world_to_grid(self, pos):
        return (
            int((pos[0] / self.resolution) + self.start_grid[0]),
            int((pos[1] / self.resolution) + self.start_grid[1])
        )

    def update_path(self, path):
        self.path = path
        self.path_index = 0
        self.following = False

    def start_following(self):
        self.following = True

    def move_along_path(self):
        if not self.following or not self.path or self.position is None:
            return

        if self.path_index + 1 >= len(self.path):
            self.stop()
            return

        target = self.path[self.path_index + 1]
        current = self.world_to_grid(self.position)

        dx = target[0] - current[0]
        dy = target[1] - current[1]

        angle_to_target = math.atan2(dy, dx)
        current_yaw = self.orientation + FRONT_OFFSET
        angle_diff = angle_to_target - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        twist = Twist()

        if abs(angle_diff) > 0.3:
            twist.angular.z = max(min(angle_diff, 1.0), -1.0)
        else:
            twist.linear.x = 0.3

        self.publisher.publish(twist)

        if math.hypot(dx, dy) < 0.5:
            self.path_index += 1

    def stop(self):
        self.publisher.publish(Twist())

# ---------------- MAIN ----------------
def main():
    rclpy.init()

    # ---------------- WAIT FOR GOAL ----------------
    goal_listener = GoalListener()
    print("⏳ Waiting for navigation goal...")

    while rclpy.ok() and goal_listener.goal_name is None:
        rclpy.spin_once(goal_listener, timeout_sec=0.1)

    goal_name = goal_listener.goal_name
    print("✅ Goal received from customer")
    print(f"➡️ Going to {goal_name}")

    # ---------------- FIXED NAVIGATION GOAL ----------------
    goal = (3, 75)

    # ---------------- LOAD MAP ----------------
    csv_grid = pd.read_csv("initial_map.csv", header=None).values.astype(np.uint8)
    csv_grid[csv_grid == 1] = OBSTACLE
    x_dim, y_dim = csv_grid.shape

    # ---------------- GUI ----------------
    gui = Animation(
        title=f"Goal received from customer\nGoing to {goal_name}",
        width=10,
        height=10,
        margin=0,
        x_dim=x_dim,
        y_dim=y_dim,
        start=(12, 45),
        goal=goal,
        viewing_range=4
    )

    gui.world.set_map(csv_grid)
    slam = SLAM(map=gui.world, view_range=4)

    follower = CmdVelController()
    msg = fetch_odometry_once(follower)
    if msg:
        follower.odom_callback(msg)

    executor = MultiThreadedExecutor()
    executor.add_node(goal_listener)
    executor.add_node(slam)
    executor.add_node(follower)

    threading.Thread(target=executor.spin, daemon=True).start()

    # ---------------- PATH PLANNING ----------------
    start_grid = follower.world_to_grid(follower.position)
    gui.set_start(start_grid)
    gui.set_position(start_grid)

    dstar = DStarLite(
        map=gui.world,
        s_start=start_grid,
        s_goal=goal
    )

    path, _, _ = dstar.move_and_replan(robot_position=start_grid)
    follower.update_path(path)

    # ---------------- MAIN LOOP ----------------
    while not gui.done:
        if follower.position:
            grid_pos = follower.world_to_grid(follower.position)
            gui.set_position(grid_pos)

        gui.run_game(path=path)

        if gui.space_pressed:
            follower.start_following()

        follower.move_along_path()

    follower.stop()
    executor.shutdown()
    rclpy.shutdown()

# ---------------- ENTRY ----------------
if __name__ == '__main__':
    main()

