import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        self.declare_parameter('max_linear_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.42)
        self.declare_parameter('goal_x', 2.2)
        self.declare_parameter('goal_y', -1.6)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('line_threshold_pixels', 5)
        self.declare_parameter('straight_speed', 0.10)
        # Minimum distance robot must travel before obstacle detection is armed
        self.declare_parameter('obstacle_arm_distance', 1.5)
        # Distance at which obstacle triggers avoidance
        self.declare_parameter('obstacle_trigger_distance', 0.55)

        self.bridge = CvBridge()

        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.obstacle_sub = self.create_subscription(Bool, '/obstacle_status', self.obstacle_callback, 10)
        self.avoid_done_sub = self.create_subscription(Bool, '/avoidance_done', self.avoidance_done_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.avoid_trigger_pub = self.create_publisher(Bool, '/avoidance_trigger', 10)

        # Mode: DRIVE_STRAIGHT -> AVOID -> FOLLOW -> DONE
        self.mode = 'DRIVE_STRAIGHT'

        self.goal_reached = False
        self.has_odom = False
        self.front_distance = float('inf')
        self.obstacle_hit_count = 0

        # Odometry tracking
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance_traveled = 0.0

        self.prev_error = 0.0
        self.avoidance_hold_until = 0.0

        self.kp = 0.6
        self.kd = 0.0

        # 20 Hz control timer - the ONLY source of cmd_vel during DRIVE_STRAIGHT
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('PathFollowerNode started — DRIVE_STRAIGHT until obstacle_arm_distance reached')

    # ------------------------------------------------------------------
    # SENSORS
    # ------------------------------------------------------------------
    def laser_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = np.inf
        ranges[ranges < max(msg.range_min, 0.10)] = np.inf

        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        # Narrow 20° cone directly in front
        front_mask = np.abs(angles) <= math.radians(20.0)
        front_ranges = ranges[front_mask]
        self.front_distance = float(np.min(front_ranges)) if front_ranges.size > 0 else float('inf')

    def odom_callback(self, msg: Odometry):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        self.has_odom = True

        # Record spawn position once
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f'Start position recorded: ({self.start_x:.2f}, {self.start_y:.2f})')

        self.distance_traveled = math.hypot(
            self.current_x - self.start_x,
            self.current_y - self.start_y
        )

        # Goal check
        if not self.goal_reached:
            goal_x = float(self.get_parameter('goal_x').value)
            goal_y = float(self.get_parameter('goal_y').value)
            goal_tol = float(self.get_parameter('goal_tolerance').value)
            if math.hypot(goal_x - self.current_x, goal_y - self.current_y) < goal_tol:
                self.goal_reached = True
                self.mode = 'DONE'
                self._publish_zero()
                self.get_logger().info('Goal reached!')

    def obstacle_callback(self, msg: Bool):
        """Only used in FOLLOW mode (after first avoidance done)."""
        if (self.mode == 'FOLLOW'
                and bool(msg.data)
                and not self.goal_reached
                and time.time() >= self.avoidance_hold_until):
            self._trigger_avoidance('obstacle_detector_node signal')

    def avoidance_done_callback(self, msg: Bool):
        if self.mode == 'AVOID' and bool(msg.data) and not self.goal_reached:
            self.mode = 'FOLLOW'
            self.prev_error = 0.0
            self.avoidance_hold_until = time.time() + 3.0
            self.get_logger().info('Avoidance done — switching to FOLLOW (camera line tracking)')

    # ------------------------------------------------------------------
    # MAIN CONTROL LOOP (20 Hz)
    # ------------------------------------------------------------------
    def control_loop(self):
        if self.goal_reached or self.mode == 'DONE':
            self._publish_zero()
            return

        if self.mode == 'AVOID':
            # avoidance_node drives the robot; we publish nothing
            return

        if self.mode == 'DRIVE_STRAIGHT':
            self._do_drive_straight()
        elif self.mode == 'FOLLOW':
            # image_callback handles FOLLOW, so nothing here
            pass

    def _do_drive_straight(self):
        """Drive perfectly straight (zero angular) until obstacle is detected.

        Obstacle detection is only armed after 'obstacle_arm_distance' metres
        have been traveled, so startup laser noise cannot trigger avoidance.
        """
        speed = float(self.get_parameter('straight_speed').value)
        arm_dist = float(self.get_parameter('obstacle_arm_distance').value)
        trigger_dist = float(self.get_parameter('obstacle_trigger_distance').value)

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0  # ALWAYS zero — no camera, no steering

        # Only check for obstacles once we've traveled enough distance
        if self.distance_traveled >= arm_dist:
            if self.front_distance < trigger_dist:
                self.obstacle_hit_count += 1
            else:
                self.obstacle_hit_count = max(0, self.obstacle_hit_count - 1)

            if self.obstacle_hit_count >= 5:
                self._trigger_avoidance('laser direct detection')
                return

            # Safety hard-stop if extremely close
            if self.front_distance < 0.28:
                twist.linear.x = 0.0
                self.get_logger().warn(f'HARD STOP: obstacle at {self.front_distance:.2f}m')

        self.cmd_pub.publish(twist)

    def _trigger_avoidance(self, reason: str = ''):
        self.mode = 'AVOID'
        self.obstacle_hit_count = 0
        self._publish_zero()
        trig = Bool()
        trig.data = True
        self.avoid_trigger_pub.publish(trig)
        self.get_logger().warn(f'AVOID triggered ({reason}) | front={self.front_distance:.2f}m | dist={self.distance_traveled:.2f}m')

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    # IMAGE CALLBACK — only active in FOLLOW mode
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        if self.goal_reached or self.mode != 'FOLLOW':
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        h, w = frame.shape[:2]
        roi = frame[int(0.30 * h):int(0.99 * h), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_black = np.array([0, 0, 0], dtype=np.uint8)
        upper_black = np.array([180, 255, 65], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_black, upper_black)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        line_pixels = cv2.countNonZero(mask)
        line_threshold = float(self.get_parameter('line_threshold_pixels').value)

        twist = Twist()
        max_lin = float(self.get_parameter('max_linear_speed').value)
        max_ang = float(self.get_parameter('max_angular_speed').value)

        if line_pixels > line_threshold:
            m = cv2.moments(mask)
            cx = int(m['m10'] / m['m00']) if m['m00'] > 0 else int(w / 2)

            error = (w / 2) - float(cx)
            norm_error = error / max(1.0, w / 2)
            d_error = norm_error - self.prev_error
            steering = norm_error * self.kp + d_error * self.kd

            twist.angular.z = float(np.clip(steering, -max_ang, max_ang))
            twist.linear.x = max_lin
            self.prev_error = norm_error
        else:
            twist.linear.x = 0.05
            twist.angular.z = 0.0

        # Brake near obstacles in FOLLOW mode
        if self.front_distance < 0.35:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif self.front_distance < 0.55:
            twist.linear.x = min(twist.linear.x, 0.03)

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
