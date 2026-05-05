#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Float32, '/line_error', self.line_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.line_error = 9999.0
        self.front_dist = 10.0

        self.state = "STRAIGHT"
        self.state_time = time.time()

        self.obstacle_check_enabled_time = time.time() + 18.0

        self.in_avoidance = False
        self.avoid_done = False

        self.line_lost_time = None

        self.get_logger().info("Controller Started")

    def line_cb(self, msg):
        self.line_error = msg.data

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = np.inf
        ranges[ranges < 0.12] = np.inf

        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        front_mask = np.abs(angles) <= math.radians(20.0)
        front_ranges = ranges[front_mask]

        self.front_dist = float(np.min(front_ranges)) if front_ranges.size > 0 else 10.0

    def set_state(self, s):
        self.state = s
        self.state_time = time.time()
        self.get_logger().info(f"STATE → {s}")

    def obstacle_detected(self):
        if self.avoid_done:
            return False

        if self.in_avoidance:
            return False

        if time.time() < self.obstacle_check_enabled_time:
            return False

        return self.front_dist < 1.1

    def control_loop(self):
        twist = Twist()
        dt = time.time() - self.state_time

        # ---------------- STRAIGHT ----------------
        if self.state == "STRAIGHT":
            twist.linear.x = 0.15

            if self.obstacle_detected():
                self.in_avoidance = True
                self.set_state("TURN_LEFT")
                return

        # ---------------- FOLLOW ----------------
        elif self.state == "FOLLOW":
            if self.obstacle_detected():
                self.in_avoidance = True
                self.set_state("TURN_LEFT")
                return

            if self.line_error == 9999.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.3
            else:
                error_norm = self.line_error / 320.0
                twist.linear.x = 0.15
                twist.angular.z = -error_norm * 0.8

        # ---------------- AVOIDANCE ----------------
        elif self.state == "TURN_LEFT":
            twist.angular.z = 0.7
            if dt > 1.3:
                self.set_state("GO_FORWARD")

        elif self.state == "GO_FORWARD":
            twist.linear.x = 0.15

            if dt > 4.5:
                self.set_state("TURN_RIGHT")

        # 🔥 FIXED TURN RIGHT (THIS WAS YOUR BUG)
        elif self.state == "TURN_RIGHT":
            twist.angular.z = -0.8   # stronger turn

            # longer duration → ensures correct alignment
            if dt > 1.8:
                self.set_state("REJOIN")

        # ---------------- REJOIN ----------------
        elif self.state == "REJOIN":

            if self.line_error != 9999.0:
                error_norm = self.line_error / 320.0

                twist.linear.x = 0.10
                twist.angular.z = -error_norm * 1.0

                if abs(self.line_error) < 20:
                    self.in_avoidance = False
                    self.avoid_done = True
                    self.obstacle_check_enabled_time = time.time() + 5.0
                    self.set_state("FOLLOW")

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.5

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = Controller()
    node.create_timer(0.1, node.control_loop)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()