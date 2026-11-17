import numpy as np
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

OBSTACLE = 255
UNOCCUPIED = 0
FRONT_OFFSET = 0.0  # adjust if needed after testing

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
    
def fetch_model_states_once(node):
    future = Future()
    def callback(msg):
        future.set_result(msg)
    sub = node.create_subscription(ModelStates, '/model_states', callback, 10)

    # Wait for the message to arrive
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    # Clean up subscription
    node.destroy_subscription(sub)
    if future.done():
        return future.result()
    else:
        print("❌ Timeout while waiting for /model_states")
        return None


class CmdVelController(Node):
    def __init__(self, robot_name='bot_name10', resolution=0.1, grid_center=(50, 50)):
        super().__init__('cmd_vel_controller')
        qos = rclpy.qos.QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', qos)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # 🔁 Manually fetch the latest /model_states once to initialize

        self.robot_name = robot_name
        self.robot_index = None
        self.position = None
        self.orientation = None
        self.resolution = resolution
        self.grid_center = grid_center
        self.path = []
        self.following = False 
        self.path_index = 0
       # self.fetch_odometry_once()
        
     
    def odom_callback(self, msg):
        try:
            self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.orientation = msg.pose.pose.orientation
            print(f"✅ Got position from odometry: {self.position}")
        except Exception as e:
            print(f"🔥 Error in odom_callback: {e}")
          
          
    def world_to_grid(self, pos):
        gx = int((pos[0] / self.resolution) + self.grid_center[0])
        gy = int((pos[1] / self.resolution) + self.grid_center[1])
        return gx, gy

    def grid_to_world(self, grid_pos):
        gx, gy = grid_pos
        wx = (gx - self.grid_center[0]) * self.resolution
        wy = (gy - self.grid_center[1]) * self.resolution
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

       # target = self.path[1]
        if self.path_index + 1 >= len(self.path):
            print("🎯 Path complete.")
            self.following = False
            self.stop()
            return

        target = self.path[self.path_index + 1]
        current_grid = self.world_to_grid(self.position)
        dx = target[0] - current_grid[0]
        dy = target[1] - current_grid[1]
        angle_to_target = math.atan2(dy, dx)
    
        #CHANGEDDD
        raw_yaw = quaternion_to_yaw(self.orientation)
# Apply front offset based on robot design
        current_yaw = raw_yaw + FRONT_OFFSET
        current_yaw = math.atan2(math.sin(current_yaw), math.cos(current_yaw))  # Normalize
        angle_diff = angle_to_target - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize
        
        # print(f"🔁 Aligning: angle_diff = {angle_diff:.3f} rad")
        print(f"🧭 angle_diff: {angle_diff:.3f} | yaw: {math.degrees(current_yaw):.2f}°")
        twist = Twist()
                
        if abs(angle_diff) < 0.05:
            angle_diff = 0.0  # dead zone to suppress wobble
        if abs(angle_diff) > 2.0:
    # Very large angle — spin only
            twist.angular.z = max(min(angle_diff, 1.0), -1.0)
            twist.linear.x = 0.0
        elif abs(angle_diff) > 0.4:
    # Moderate misalignment — rotate + nudge forward
            twist.angular.z = max(min(angle_diff, 0.5), -0.5)
            twist.linear.x = 0.08
        else:
    # Close enough — move forward with gentle steer
            twist.angular.z = 0.5 * angle_diff  # dampen the correction
            twist.linear.x = 0.18


 
        self.publisher.publish(twist)
        print(f"📤 Publishing Twist → linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")
        
        # Check if we reached this waypoint
        dx = target[0] - current_grid[0]
        dy = target[1] - current_grid[1]

        distance = math.hypot(dx, dy)
        print(f"📏 Distance to next point: {distance:.2f}")
        if distance < 0.6:
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

    x_dim, y_dim = 100, 100
    goal = (60, 60)
    view_range = 5

    new_map = OccupancyGridMap(x_dim=x_dim, y_dim=y_dim)
    load_static_grid(new_map)
    gui = Animation(title="D* Lite Path Planning", width=7, height=7, margin=0,
                    x_dim=x_dim, y_dim=y_dim, start=(0, 0), goal=goal, viewing_range=view_range)
    gui.world = new_map

    slam = SLAM(map=new_map, view_range=view_range)
    slam.slam_map.set_map(new_map.get_map().copy())  # 👈 Pass static obstacles into SLAM map
    gui.world = slam.slam_map  # ✅ Tell the GUI to display the SLAM-updated map


    # Fetch and apply initial static obstacles
    #model_states_node = rclpy.create_node("temp_model_states_node")
    #msg = fetch_model_states_once(model_states_node)
    #if msg:
    #    slam.model_states_callback(msg)
    #else:
    #    print("⚠️ Failed to fetch /model_states")
   # model_states_node.destroy_node()

                   
    follower = CmdVelController(robot_name="bot_name10", resolution=0.1, grid_center=(x_dim // 2, y_dim // 2))
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

   # populate_grid_from_gazebo(grid=new_map, slam=slam)

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

    dstar = DStarLite(map=slam.slam_map, s_start=start_grid, s_goal=goal)
    #initial_vertices, slam_map = slam.rescan(global_position=start_grid)
    #dstar.new_edges_and_old_costs = initial_vertices
    dstar.sensed_map = slam.slam_map
    #gui.world = slam_map

    path, g, rhs = dstar.move_and_replan(robot_position=start_grid)
    follower.update_path(path)

    while not gui.done:
        if follower.position is not None:
            grid_pos = follower.world_to_grid(follower.position)
            #print("📍 Robot position in world:", follower.position, "→ grid:", grid_pos)
        print(f"📍 Robot world position: x={follower.position[0]:.3f}, y={follower.position[1]:.3f} → grid={grid_pos}")
        gui.set_position(grid_pos)

           #gui.set_position(follower.world_to_grid(follower.position))
        gui.run_game(path=path[follower.path_index:])

        if gui.space_pressed:
            follower.start_following()

        follower.move_along_path()
        # Add after follower.move_along_path()

        if follower.position:
            new_pos = follower.world_to_grid(follower.position)
            print(f"🔁 SLAM rescanning at position: {new_pos}")
            if new_pos != start_grid:
                dstar.s_start = new_pos        
        # 📸 Replace this with actual image capture when deploying on robot
                if slam.depth_image is not None:
                    updated_vertices = slam.process_kinect_obstacles(slam.depth_image)
                else:
                    updated_vertices = None
                    print("⚠️ Depth image missing — skipping update")
                
                if updated_vertices:
                    dstar.new_edges_and_old_costs = updated_vertices
                    dstar.sensed_map = slam.slam_map
                    path, g, rhs = dstar.move_and_replan(robot_position=new_pos)
                    follower.update_path(path)
                    #follower.start_following()
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
