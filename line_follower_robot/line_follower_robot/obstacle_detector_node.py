import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.declare_parameter('obstacle_distance', 0.5)
        self.declare_parameter('forward_half_angle_deg', 30.0)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.status_pub = self.create_publisher(Bool, '/obstacle_status', 10)

        self.last_status = False
        self.get_logger().info('Obstacle detector node started')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = np.inf
        ranges[ranges < max(msg.range_min, 0.08)] = np.inf

        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        half_angle = math.radians(float(self.get_parameter('forward_half_angle_deg').value))
        forward_mask = np.abs(angles) <= half_angle

        forward_ranges = ranges[forward_mask]
        forward_min = float(np.min(forward_ranges)) if forward_ranges.size > 0 else float('inf')

        obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        status = forward_min <= obstacle_distance

        msg_out = Bool()
        msg_out.data = status
        self.status_pub.publish(msg_out)

        if status != self.last_status:
            if status:
                self.get_logger().warn(f'Obstacle detected at {forward_min:.2f} m')
            else:
                self.get_logger().info('Obstacle cleared')
            self.last_status = status


def main():
    rclpy.init()
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
