# generate_static_map.py

import numpy as np
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from grid import OccupancyGridMap, SLAM

class StaticMapGenerator(Node):
    def __init__(self, x_dim=100, y_dim=100, resolution=0.1, robot_name="bot_name10", output_file="initial_map.csv"):
        super().__init__('static_map_generator')
        self.grid = OccupancyGridMap(x_dim, y_dim)
        self.slam = SLAM(map=self.grid, view_range=0)  # No SLAM needed, just grabbing models
        self.resolution = resolution
        self.output_file = output_file
        self.robot_name = robot_name
        self.subscription = self.create_subscription(ModelStates, '/model_states', self.model_callback, 10)

    def model_callback(self, msg):
        center_x = self.grid.x_dim // 2
        center_y = self.grid.y_dim // 2

        for i, name in enumerate(msg.name):
            if name in ["ground_plane", self.robot_name]:
                continue
            x = msg.pose[i].position.x
            y = msg.pose[i].position.y
            gx = int(x / self.resolution + center_x)
            gy = int(y / self.resolution + center_y)
            if 0 <= gx < self.grid.x_dim and 0 <= gy < self.grid.y_dim:
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        ox, oy = gx + dx, gy + dy
                        if 0 <= ox < self.grid.x_dim and 0 <= oy < self.grid.y_dim:
                            self.grid.set_obstacle((ox, oy))

        np.savetxt(self.output_file, self.grid.get_map(), fmt='%d', delimiter=",")
        print(f"✅ Map saved to {self.output_file}")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = StaticMapGenerator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

