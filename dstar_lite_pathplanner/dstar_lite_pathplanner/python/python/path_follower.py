import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import csv

class PathFollower(Node):
    def __init__(self, resolution):
        super().__init__('path_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/path_cmd', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path = []  # Empty path initially
        self.resolution = resolution
        self.index = 0
        self.current_pose = None
        self.timer = self.create_timer(0.1, self.follow_path)

    def path_callback(self, msg):
        """Converts string data to x, y coordinates"""
        path_point = msg.data.split(",")  # Split the string by comma
        x, y = float(path_point[0]), float(path_point[1])
        self.path.append((x, y))  # Append the path as a tuple of floats

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def follow_path(self):
        if self.current_pose is None or self.index >= len(self.path):
            return

        target = self.path[self.index]
        tx = target[0] * self.resolution
        ty = target[1] * self.resolution

        dx = tx - self.current_pose.position.x
        dy = ty - self.current_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        angle = math.atan2(dy, dx)  # Angle to the target point
        current_angle = self.current_pose.orientation.z  # Assume z is the yaw or theta

        # Decide the direction
        if dist < 0.1:
            self.index += 1
            return

        cmd = Twist()

        # If the angle difference is small, move straight
        angle_diff = angle - current_angle
        if abs(angle_diff) < 0.1:  # Threshold for straight movement
            self.publish_command('W')  # Move straight (W)
        elif angle_diff > 0.1:  # If robot needs to turn left
            self.publish_command('A')  # Turn left (A)
        elif angle_diff < -0.1:  # If robot needs to turn right
            self.publish_command('D')  # Turn right (D)

    def publish_command(self, command):
        """Publish a control command (W, A, D, S) to the controller"""
        command_msg = String()
        command_msg.data = command
        self.cmd_pub.publish(command_msg)
        self.get_logger().info(f"Published command: {command}")


def main(args=None):
    rclpy.init(args=args)

    resolution = 0.1  # meters per grid cell
    node = PathFollower(resolution)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

