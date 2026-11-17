import numpy as np
import pandas as pd
import time
import rclpy
import math
import os
import threading
from gui import Animation
from d_star_lite import DStarLite
from grid import OccupancyGridMap, SLAM
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.task import Future
from rclpy.node import Node
from grid import OBSTACLE


OBSTACLE = 255
UNOCCUPIED = 0
FRONT_OFFSET = math.radians(0.0)
  # adjust if needed after testing

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
    
def fetch_odometry_once(node: Node, timeout_sec=2.0):
    future = Future()
    def one_time_cb(msg):
        future.set_result(msg)

    sub = node.create_subscription(Odometry, '/odom', one_time_cb, 10)
    start = time.time()
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > timeout_sec:
            print("❌ Timeout waiting for odom")
            break

    node.destroy_subscription(sub)
    return future.result() if future.done() else None


class CmdVelController(Node):
    def __init__(self, robot_name='bot_name10', resolution=0.5, grid_center=(8, 43)):
        super().__init__('cmd_vel_controller')
        qos = rclpy.qos.QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', qos)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
          
        self.robot_name = robot_name
        self.robot_index = None
        self.position = None
        self.orientation = None
        self.resolution = resolution
        self.start_grid = (12,45)# change when start grid position u are changing
        self.path = []
        self.following = False 
        self.path_index = 0
      

        
    def odom_callback(self, msg):
        try:
        
            self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.orientation = msg.pose.pose.orientation.z
           # print(f"✅ Got position from odometry: {self.position}")
            #print(f"[ODOM Callback] Quaternion: x={self.orientation.x:.3f}, y={self.orientation.y:.3f}, z={self.orientation.z:.3f},         w={self.orientation.w:.3f}")
        except Exception as e:
            print(f"🔥 Error in odom_callback: {e}")
        
          

    def world_to_grid(self, pos):
        gx = int((pos[0] / self.resolution) + self.start_grid[0])
        gy = int((pos[1] / self.resolution) + self.start_grid[1])
        return gx, gy

    def grid_to_world(self, grid_pos):
        gx, gy = grid_pos
        wx = (gx - self.start_grid[0]) * self.resolution
        wy = (gy - self.start_grid[1]) * self.resolution
        return wx, wy

    def update_path(self, path):
        self.path = path
        #self.path_index = 0
        self.following = False

    def start_following(self):
        self.following = True

    def move_along_path(self):
        print(f"🚗 Trying to follow path. following={self.following}, path_index={self.path_index}/{len(self.path)}")

        if not self.following or not self.path or len(self.path) < 2 or self.position is None:
            return


        if self.path_index + 1 >= len(self.path):
            print("🎯 Path complete.")
            self.following = False
            self.stop()
            return

        target = self.path[self.path_index + 1]
        current_grid = self.world_to_grid(self.position)
        dx = -target[0] + current_grid[0]
        dy = -target[1] + current_grid[1]
        print(f"[DEBUG] Current grid position: {current_grid}")
        print(f"[DEBUG] Target grid position: {target}")
        print(f"[DEBUG] dx: {dx}, dy: {dy}")
        angle_to_target = math.atan2(dy, dx)
       # angle_to_target = 1.571
    
        #CHANGEDDD
        raw_yaw = self.orientation
# Apply front offset based on robot design
        current_yaw = raw_yaw + FRONT_OFFSET
        current_yaw = math.atan2(math.sin(current_yaw), math.cos(current_yaw))  # Normalize
        angle_diff = angle_to_target - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize
        #angle_diff = 1.571
        
        # print(f"🔁 Aligning: angle_diff = {angle_diff:.3f} rad")
        print(f"🧭 angle_diff: {angle_diff:.3f} | yaw: {math.degrees(current_yaw):.2f}°")
        twist = Twist()
                
        if abs(angle_diff) < 0.15:
            angle_diff = 0.0  # dead zone to suppress wobble

        if abs(angle_diff) > 0.3:
    # Not aligned: rotate in place
            twist.angular.z = max(min(angle_diff, 1.0), -1.0)
            twist.linear.x = 0.0
        else:
    # Aligned enough: move straight
            twist.angular.z = 0.0
            twist.linear.x = 0.30  # or your desired forward speed
 
        self.publisher.publish(twist)
        print(f"📤 Publishing Twist → linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")
        
        # Check if we reached this waypoint
        dx = target[0] - current_grid[0]
        dy = target[1] - current_grid[1]

        distance = math.hypot(dx, dy)
        print(f"📏 Distance to next point: {distance:.2f}")
        if distance < 0.5:
            self.path_index += 1
            if hasattr(self, "target_heading"):
                del self.target_heading
   


    def stop(self):
        self.publisher.publish(Twist())


def load_static_grid(grid: OccupancyGridMap, csv_path="initial_map.csv"):
    if not os.path.exists(csv_path):
        print(f"❌ Grid file not found: {csv_path}")
        return
    loaded = np.loadtxt(csv_path, delimiter=",", dtype=np.uint8)
    grid.set_map(loaded)
    print(f"✅ Loaded static grid from {csv_path}")



def main():
    rclpy.init()
    
    # Load CSV grid
    csv_grid = pd.read_csv("initial_map.csv", header=None).values.astype(np.uint8)

    # Convert 1 → OBSTACLE (255)
    csv_grid[csv_grid == 1] = OBSTACLE

    # Get shape from CSV
    x_dim, y_dim = csv_grid.shape
    start_grid = (12, 45)
    start = start_grid
    goal = (3, 45)
    view_range = 4

   # new_map = OccupancyGridMap(x_dim=x_dim, y_dim=y_dim)
   # load_static_grid(new_map)
    gui = Animation(title="D* Lite Path Planning", width=10, height=10, margin=0,
                    x_dim=x_dim, y_dim=y_dim, start=start, goal=goal, viewing_range=view_range)
    gui.world.set_map(csv_grid)
    new_map = gui.world
    old_map = new_map

   # new_position = start
   # last_position = start
  #  gui.world = new_map

    slam = SLAM(map=new_map, view_range=view_range)
    follower = CmdVelController(robot_name="bot_name10", resolution=0.5, grid_center=(x_dim // 2, y_dim // 2))
    # Manually fetch one odometry message to initialize position
    msg = fetch_odometry_once(follower)
    if msg:
        follower.odom_callback(msg)
    else:
        print("❌ Could not fetch initial odometry data")

    executor = MultiThreadedExecutor()
    executor.add_node(slam)
    executor.add_node(follower)
    threading.Thread(target=executor.spin, daemon=True).start()


    print("⏳ Waiting for /odom position...")
    timeout_counter = 0
    while follower.position is None:
        rclpy.spin_once(follower, timeout_sec=0.1)
        timeout_counter += 1
        if timeout_counter > 150:  # ~15 seconds
            print("❌ Timeout: No odometry data received. Exiting.")
            return
        print(f"✅ Got position from odometry: {follower.position}")

    start_grid = follower.world_to_grid(follower.position)
    gui.set_start(start_grid)
    gui.set_position(start_grid)

    dstar = DStarLite(map=new_map, s_start=start_grid, s_goal=goal)
    initial_vertices, slam_map = slam.rescan(global_position=start_grid)
    dstar.new_edges_and_old_costs = initial_vertices
    dstar.sensed_map = slam_map
    #gui.world = slam_map

    path, g, rhs = dstar.move_and_replan(robot_position=start)
    follower.update_path(path)

    while not gui.done:
        if follower.position is not None:
            grid_pos = follower.world_to_grid(follower.position)
            print("📍 Robot position in world:", follower.position, "→ grid:", grid_pos)
            gui.set_position(grid_pos)

           #gui.set_position(follower.world_to_grid(follower.position))
        gui.run_game(path=path)

        if gui.space_pressed:
            follower.start_following()

        follower.move_along_path()
        # Add after follower.move_along_path()

        if follower.position:
            new_pos = follower.world_to_grid(follower.position)

            if new_pos != start_grid:
                dstar.s_start = new_pos
                updated_vertices, updated_map = slam.rescan(global_position=new_pos)
                dstar.sensed_map = updated_map
                dstar.new_edges_and_old_costs = updated_vertices
                path, g, rhs = dstar.move_and_replan(robot_position=new_pos)
                follower.update_path(path)
                start_grid = new_pos
    # Goal reached check
        gx, gy = goal
        nx, ny = new_pos
        if abs(gx - nx) <= 1 and abs(gy - ny) <= 1:
            print("🎯 Goal reached by real bot!")
            gui.done = True

    follower.stop()
    executor.shutdown()
    slam.destroy_node()
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
