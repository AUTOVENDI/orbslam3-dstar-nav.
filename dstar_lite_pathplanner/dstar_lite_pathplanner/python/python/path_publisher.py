import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(String, '/path_cmd', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

        # Path from the D* Lite algorithm (can be generated or preloaded)
        self.path = []
        with open('planned_path.csv', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                self.path.append(f"{row[0]},{row[1]}")  # Add path points

    def publish_path(self):
        for point in self.path:
            msg = String()
            msg.data = point
            self.publisher.publish(msg)
            self.get_logger().info(f"Published path: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

