import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import time

class TwistToCmdVel(Node):
    def __init__(self):
        super().__init__('twist_to_cmd')
        
        # Create a subscriber to listen to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.twist_callback, 
            10
        )
        self.log_file = open('cmd_vel_log.txt', 'a')  # Append mode

        # Publisher to send commands (e.g., W, A, S, D)
        self.publisher = self.create_publisher(String, '/controller_cmd', 10)
    
    def twist_callback(self, msg):
        # Extract linear and angular velocity from the Twist message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        log_line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} | linear_x: {linear_velocity:.3f}, angular_z: {angular_velocity:.3f}\n"
        self.log_file.write(log_line)
        self.log_file.flush()  # Ensure it's written immediately

        
        move_time = 0.5  # seconds to move before stopping
        move_time2 = 0.5  # s

        if linear_velocity > 0.01:
            self.publish_command('W')  # Move forward
         #   time.sleep(move_time)
           # self.publish_command('S')  # Stop after moving
           # time.sleep(move_time2)
        elif linear_velocity < -0.01:
            self.publish_command('X')  # Move backward
          #  time.sleep(move_time)
          #  self.publish_command('S')  # Stop after moving
          #  time.sleep(move_time2)
        elif angular_velocity > 0.1:
            self.publish_command('D')  # Turn right
           # time.sleep(move_time)
           # self.publish_command('S')  # Stop after turning
           # time.sleep(move_time2)
        elif angular_velocity < -0.1:
            self.publish_command('A')  # Turn left
          #  time.sleep(move_time)
          #  self.publish_command('S')  # Stop after turning
          #  time.sleep(move_time2)
        else:
            self.publish_command('S')  # Stay stopped
        
    def publish_command(self, command):
        # Publish the corresponding command to your robot or controller
        command_msg = String()
        command_msg.data = command
        self.publisher.publish(command_msg)
        self.get_logger().info(f"Published command: {command}")
      #  time.sleep(0.3)  # <-- Add a small delay here (e.g., 300 milliseconds)


def main(args=None):
    rclpy.init(args=args)
    
    node = TwistToCmdVel()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

