import math
import time
from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance_node')

        self.declare_parameter('turn_angle_deg', 45.0)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('bypass_speed', 0.08)
        self.declare_parameter('advance_speed', 0.10)
        self.declare_parameter('parallel_distance', 0.95)
        self.declare_parameter('line_threshold_pixels', 120)
        self.declare_parameter('line_rejoin_stable_cycles', 4)
        self.declare_parameter('waypoints', [0.0, -0.2, 2.2, -0.2, 2.2, -1.6])
        self.declare_parameter('waypoint_reached_radius', 0.28)

        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, '/avoidance_done', 10)

        self.trigger_sub = self.create_subscription(Bool, '/avoidance_trigger', self.trigger_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.active = False
        self.phase = 'IDLE'
        self.phase_start_time = time.time()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False

        self.left_clearance = float('inf')
        self.right_clearance = float('inf')
        self.turn_sign = 1.0
        self.entry_turn_sign = 1.0
        self.return_turn_sign = -1.0
        self.line_detected = False
        self.line_stable_count = 0
        self.turn_stage = 'ENTRY'

        self.start_heading = 0.0
        self.target_heading = 0.0
        self.path_heading = 0.0
        self.step_start_x = 0.0
        self.step_start_y = 0.0

        self.waypoints = self._parse_waypoints(self.get_parameter('waypoints').value)
        self.waypoint_idx = 0

        self.get_logger().info('Avoidance node started')

    def _parse_waypoints(self, raw: List[float]) -> List[Tuple[float, float]]:
        data = [float(v) for v in raw]
        if len(data) < 2:
            return [(2.2, -1.6)]
        if len(data) % 2 == 1:
            data = data[:-1]
        return [(data[i], data[i + 1]) for i in range(0, len(data), 2)]

    def trigger_callback(self, msg: Bool):
        if not msg.data or self.active or not self.has_odom:
            return

        self.active = True
        self.phase = 'STOP'
        self.phase_start_time = time.time()
        self.start_heading = self.current_yaw
        self.path_heading = self._heading_to_next_waypoint()
        self.entry_turn_sign = 1.0
        self.return_turn_sign = -1.0
        self.turn_stage = 'ENTRY'
        self.line_stable_count = 0
        self._publish_zero()
        self.get_logger().warn('Avoidance triggered')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = np.inf
        ranges[ranges < max(msg.range_min, 0.08)] = np.inf

        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        left_mask = (angles >= math.radians(20.0)) & (angles <= math.radians(100.0))
        right_mask = (angles <= math.radians(-20.0)) & (angles >= math.radians(-100.0))

        self.left_clearance = float(np.min(ranges[left_mask])) if np.any(left_mask) else float('inf')
        self.right_clearance = float(np.min(ranges[right_mask])) if np.any(right_mask) else float('inf')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        h, _ = frame.shape[:2]
        roi = frame[int(0.60 * h):int(0.98 * h), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, np.array([0, 0, 0], dtype=np.uint8), np.array([180, 255, 70], dtype=np.uint8))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        line_pixels = cv2.countNonZero(mask)
        self.line_detected = line_pixels >= int(self.get_parameter('line_threshold_pixels').value)
        if self.line_detected:
            self.line_stable_count += 1
        else:
            self.line_stable_count = 0

    def odom_callback(self, msg: Odometry):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

        reach_r = float(self.get_parameter('waypoint_reached_radius').value)
        while self.waypoint_idx < len(self.waypoints):
            wx, wy = self.waypoints[self.waypoint_idx]
            if math.hypot(wx - self.current_x, wy - self.current_y) <= reach_r:
                self.waypoint_idx += 1
            else:
                break

    def _set_phase(self, phase: str):
        self.phase = phase
        self.phase_start_time = time.time()
        self.step_start_x = self.current_x
        self.step_start_y = self.current_y
        self.get_logger().info(f'Avoidance step: {phase}')

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _distance_from_step_start(self) -> float:
        return math.hypot(self.current_x - self.step_start_x, self.current_y - self.step_start_y)

    def _next_waypoint(self) -> Tuple[float, float]:
        if not self.waypoints:
            return (self.current_x + math.cos(self.current_yaw), self.current_y + math.sin(self.current_yaw))
        idx = min(self.waypoint_idx, len(self.waypoints) - 1)
        return self.waypoints[idx]

    def _heading_to_next_waypoint(self) -> float:
        wx, wy = self._next_waypoint()
        return math.atan2(wy - self.current_y, wx - self.current_x)

    def _is_heading_reached(self, target: float, tol: float = 0.06) -> bool:
        return abs(normalize_angle(target - self.current_yaw)) < tol

    def control_loop(self):
        if not self.active:
            return

        now = time.time()
        twist = Twist()

        turn_angle = math.radians(float(self.get_parameter('turn_angle_deg').value))
        turn_speed = float(self.get_parameter('turn_speed').value)
        bypass_speed = float(self.get_parameter('bypass_speed').value)
        advance_speed = float(self.get_parameter('advance_speed').value)
        parallel_distance = float(self.get_parameter('parallel_distance').value)

        if self.phase == 'STOP':
            if now - self.phase_start_time >= 0.5:
                self._set_phase('SCAN')
            self._publish_zero()
            return

        if self.phase == 'SCAN':
            # Always turn left (robot's left = +X in world when robot faces -Y)
            # This makes the robot go to the right of the obstacle as seen from above
            self.turn_sign = 1.0
            self.entry_turn_sign = self.turn_sign
            self.return_turn_sign = -self.turn_sign
            self.target_heading = normalize_angle(self.start_heading + self.turn_sign * turn_angle)
            self._set_phase('TURN_LEFT_45')

        elif self.phase in ('TURN_LEFT_45', 'TURN_RIGHT_45'):
            if self.turn_stage == 'ENTRY':
                twist.angular.z = self.entry_turn_sign * turn_speed
            else:
                twist.angular.z = self.return_turn_sign * turn_speed
            if self._is_heading_reached(self.target_heading):
                if self.turn_stage == 'ENTRY':
                    self._set_phase('GO_PARALLEL')
                else:
                    self._set_phase('REJOIN_LINE')

        elif self.phase == 'GO_PARALLEL':
            twist.linear.x = bypass_speed
            if self._distance_from_step_start() >= parallel_distance:
                self.turn_stage = 'RETURN'
                self.target_heading = self.start_heading
                self._set_phase('TURN_RIGHT_45' if self.turn_sign > 0.0 else 'TURN_LEFT_45')

        elif self.phase == 'REJOIN_LINE':
            err = normalize_angle(self.path_heading - self.current_yaw)
            twist.linear.x = max(advance_speed, 0.11)
            twist.angular.z = float(np.clip(err * 0.8, -0.25, 0.25))

            if self.line_detected:
                if self.line_stable_count >= int(self.get_parameter('line_rejoin_stable_cycles').value):
                    self._set_phase('RESUME')
            else:
                self.line_stable_count = 0

        elif self.phase == 'RESUME':
            self._publish_zero()
            done = Bool()
            done.data = True
            self.done_pub.publish(done)
            self.active = False
            self.phase = 'IDLE'
            self.get_logger().info('Avoidance done, resuming line follow')
            return

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = AvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
