import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('obstacle_distance', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('turn_direction', 'left')  # 'left' or 'right'

        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Obstacle Detector Node Started")
        self.obstacle_detected = False

    def laser_callback(self, msg):
        # Get parameters
        obstacle_distance = self.get_parameter('obstacle_distance').value
        max_angular_speed = self.get_parameter('max_angular_speed').value
        turn_direction = self.get_parameter('turn_direction').value

        # Extract ranges (filter out inf and nan values)
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges) | np.isnan(ranges)] = np.inf

        # Check different angular regions
        # Left front (45-90 degrees)
        left_front_start = len(ranges) // 4
        left_front_end = int(len(ranges) * 3 / 8)
        left_min = np.min(ranges[left_front_start:left_front_end])

        # Front center (170-190 degrees or indices 170-190)
        front_start = int(len(ranges) * 170 / 360)
        front_end = int(len(ranges) * 190 / 360)
        front_min = np.min(ranges[front_start:front_end])

        # Right front (270-315 degrees)
        right_front_start = int(len(ranges) * 5 / 8)
        right_front_end = int(len(ranges) * 3 / 4)
        right_min = np.min(ranges[right_front_start:right_front_end])

        twist = Twist()

        # Obstacle avoidance logic
        if front_min < obstacle_distance:
            self.get_logger().warn(f"Obstacle detected at {front_min:.2f}m. Obstacle Avoidance activated!")
            self.obstacle_detected = True

            # Stop moving forward
            twist.linear.x = 0.0

            # Turn away from obstacle based on direction parameter
            if turn_direction == 'left':
                twist.angular.z = max_angular_speed
            else:
                twist.angular.z = -max_angular_speed

        else:
            # No obstacle - allow normal line following
            self.obstacle_detected = False

        # Publish velocity command
        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

