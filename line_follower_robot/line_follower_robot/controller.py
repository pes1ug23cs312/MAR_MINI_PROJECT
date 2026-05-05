import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


class LineFollowerController(Node):
    def __init__(self):
        super().__init__('line_follower_controller')

        self.declare_parameter('obstacle_threshold', 0.62)
        self.declare_parameter('line_threshold', 140)
        self.declare_parameter('max_linear_speed', 0.11)
        self.declare_parameter('max_angular_speed', 0.28)
        self.declare_parameter('min_line_follow_time_before_avoid', 6.0)

        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/controller/debug', 10)

        self.bridge = CvBridge()

        self.front_distance = float('inf')
        self.prev_error = 0.0
        self.last_line_time = time.time()
        self.last_steer_sign = 1.0
        self.task_finished = False

        self.start_time = time.time()
        self.line_follow_started_at = None
        self.obstacle_cooldown_until = 0.0
        self.avoid_rearm_until = 0.0
        self.avoid_completed_once = False
        self.emergency_brake_until = 0.0

        self.obstacle_hit_count = 0
        self.rejoin_line_hits = 0
        self.turn_in_line_hits = 0

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.kp = 0.45
        self.kd = 0.06

        # FOLLOW_LINE | AVOID_STOP | AVOID_TURN_LEFT | AVOID_FORWARD_PASS | AVOID_FORWARD_CLEAR
        # AVOID_TURN_RIGHT_SEARCH | AVOID_FORWARD_FIND_LINE | AVOID_REJOIN | AVOID_STRAIGHTEN | FINISHED
        self.state = 'FOLLOW_LINE'
        self.state_start = time.time()
        self.mode = 'LINE_FOLLOW'

        self.get_logger().info('=' * 50)
        self.get_logger().info('Line Follower Controller Started!')
        self.get_logger().info('=' * 50)

    def _set_state(self, new_state: str):
        self.state = new_state
        self.state_start = time.time()
        self.get_logger().info(f'State -> {new_state}')

    def _line_from_camera(self, frame):
        h, _ = frame.shape[:2]
        roi_top = 0.55 if self.state == 'FOLLOW_LINE' else 0.35
        roi = frame[int(h * roi_top):int(h * 0.95), :]
        roi_h, roi_w = roi.shape[:2]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        line_pixels = cv2.countNonZero(mask)
        base_threshold = float(self.get_parameter('line_threshold').value)
        detect_threshold = base_threshold if self.state == 'FOLLOW_LINE' else max(60.0, base_threshold * 0.50)
        line_detected = line_pixels > detect_threshold

        cx = int(roi_w / 2)
        if line_detected:
            m = cv2.moments(mask)
            if m['m00'] > 0:
                cx = int(m['m10'] / m['m00'])

        error = (roi_w / 2) - cx
        return mask, roi_w, roi_h, cx, error, line_detected, line_pixels

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Error converting image: {exc}')
            return

        mask, roi_w, roi_h, cx, error, line_detected, line_pixels = self._line_from_camera(frame)
        twist = Twist()
        now = time.time()
        base_threshold = float(self.get_parameter('line_threshold').value)

        if self.state == 'FINISHED' or self.task_finished:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.mode = 'FINISHED'
            self.cmd_vel_pub.publish(twist)
            return

        if self.state == 'AVOID_STOP':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.mode = 'OBSTACLE_AVOID'
            if now - self.state_start > 0.45:
                self._set_state('AVOID_TURN_LEFT')

        elif self.state == 'AVOID_TURN_LEFT':
            # Turn left first.
            twist.linear.x = 0.0
            twist.angular.z = 0.48
            self.mode = 'OBSTACLE_AVOID'
            if now - self.state_start > 1.45:
                self._set_state('AVOID_FORWARD_PASS')

        elif self.state == 'AVOID_FORWARD_PASS':
            # Go straight for sufficient distance to pass obstacle.
            twist.linear.x = 0.09
            twist.angular.z = 0.0
            self.mode = 'OBSTACLE_AVOID'
            if now - self.state_start > 6.80:
                self._set_state('AVOID_FORWARD_CLEAR')

        elif self.state == 'AVOID_FORWARD_CLEAR':
            # Go extra straight distance so turn-back won't hit obstacle.
            twist.linear.x = 0.08
            twist.angular.z = 0.0
            self.mode = 'OBSTACLE_AVOID'
            if now - self.state_start > 3.40:
                self.turn_in_line_hits = 0
                self._set_state('AVOID_TURN_RIGHT_SEARCH')

        elif self.state == 'AVOID_TURN_RIGHT_SEARCH':
            # Single right turn to aim back toward the line area.
            twist.linear.x = 0.01
            twist.angular.z = -0.42
            self.mode = 'OBSTACLE_AVOID'
            if now - self.state_start > 1.55:
                self.turn_in_line_hits = 0
                self._set_state('AVOID_FORWARD_FIND_LINE')

        elif self.state == 'AVOID_FORWARD_FIND_LINE':
            # After right turn, keep moving forward until a valid black-line signal is seen.
            elapsed = now - self.state_start
            twist.linear.x = 0.07
            twist.angular.z = -0.06
            self.mode = 'OBSTACLE_AVOID'

            roi_area = max(1.0, float(roi_w * roi_h))
            line_valid = line_detected and (line_pixels > (base_threshold * 0.90)) and (line_pixels < (roi_area * 0.18))

            if line_valid:
                self.turn_in_line_hits += 1
            else:
                self.turn_in_line_hits = 0

            # Minimum forward travel + stable detections before merge-to-line phase.
            if elapsed > 2.60 and self.turn_in_line_hits >= 8:
                self.rejoin_line_hits = 0
                self._set_state('AVOID_REJOIN')
            elif elapsed > 9.50:
                # Safety fallback: attempt merge with camera guidance.
                self.rejoin_line_hits = 0
                self._set_state('AVOID_REJOIN')

        elif self.state == 'AVOID_REJOIN':
            # Move toward the black line and merge onto it using camera guidance.
            roi_area = max(1.0, float(roi_w * roi_h))
            line_confident = line_detected and (line_pixels > (base_threshold * 0.90)) and (line_pixels < (roi_area * 0.18))
            if line_confident:
                norm_error = float(error / max(1.0, roi_w / 2.0))
                d_error = norm_error - self.prev_error
                steering = (norm_error * self.kp + d_error * self.kd)
                twist.angular.z = float(np.clip(steering, -0.30, 0.30))
                twist.linear.x = 0.055
                self.prev_error = norm_error

                if abs(error) < 10:
                    self.rejoin_line_hits += 1
                else:
                    self.rejoin_line_hits = 0

                if self.rejoin_line_hits >= 10:
                    self.turn_in_line_hits = 0
                    self._set_state('AVOID_STRAIGHTEN')
            else:
                # If line is partially visible, slow down and align to center before advancing.
                if line_detected:
                    norm_error = float(error / max(1.0, roi_w / 2.0))
                    # Pause-forward behavior prevents crossing past the line before locking center.
                    twist.linear.x = 0.0 if abs(error) > 18 else 0.02
                    twist.angular.z = float(np.clip(norm_error * 0.35, -0.24, 0.24))
                else:
                    # Continue forward with mild right bias until line is reacquired.
                    twist.linear.x = 0.05
                    twist.angular.z = -0.07
            self.mode = 'OBSTACLE_AVOID'

        elif self.state == 'AVOID_STRAIGHTEN':
            # On black line: align straight first, then and only then switch to normal line tracking.
            roi_area = max(1.0, float(roi_w * roi_h))
            line_confident = line_detected and (line_pixels > (base_threshold * 0.90)) and (line_pixels < (roi_area * 0.18))
            if line_confident:
                norm_error = float(error / max(1.0, roi_w / 2.0))
                twist.linear.x = 0.065
                twist.angular.z = float(np.clip(norm_error * 0.15, -0.07, 0.07))
                if abs(error) < 5:
                    self.turn_in_line_hits += 1
                else:
                    self.turn_in_line_hits = 0
            else:
                twist.linear.x = 0.05
                twist.angular.z = -0.08
                self.turn_in_line_hits = 0

            self.mode = 'OBSTACLE_AVOID'
            if now - self.state_start > 2.80 and self.turn_in_line_hits >= 16:
                self._set_state('FOLLOW_LINE')
                # Hold off obstacle retrigger right after rejoin to avoid re-entering avoid loop.
                self.obstacle_cooldown_until = time.time() + 10.0
                self.avoid_rearm_until = time.time() + 8.0
                self.obstacle_hit_count = 0
                self.avoid_completed_once = True
                self.prev_error = 0.0

        else:  # FOLLOW_LINE
            if line_detected:
                norm_error = float(error / max(1.0, roi_w / 2.0))
                d_error = norm_error - self.prev_error
                steering = (norm_error * self.kp + d_error * self.kd)
                if abs(norm_error) < 0.02:
                    steering = 0.0

                follow_ang_limit = max(0.14, self.max_angular_speed * 0.55)
                twist.angular.z = float(np.clip(steering, -follow_ang_limit, follow_ang_limit))
                speed_scale = max(0.55, 1.0 - 0.55 * abs(norm_error))
                twist.linear.x = self.max_linear_speed * speed_scale

                obstacle_threshold = float(self.get_parameter('obstacle_threshold').value)
                if self.front_distance < (obstacle_threshold + 0.20):
                    twist.linear.x = min(twist.linear.x, 0.05)

                self.prev_error = norm_error
                self.last_line_time = time.time()
                self.last_steer_sign = 1.0 if twist.angular.z >= 0.0 else -1.0
                self.mode = 'LINE_FOLLOW'
                if self.line_follow_started_at is None:
                    self.line_follow_started_at = time.time()
                self.get_logger().info(
                    f'LINE FOLLOW | Speed: {twist.linear.x:.2f} | Steer: {twist.angular.z:.2f} | Err: {error:.1f}'
                )
            else:
                dt = time.time() - self.last_line_time
                # At project end: after avoidance is completed, if line is lost for long, stop permanently.
                if self.avoid_completed_once and dt > 1.0:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.task_finished = True
                    self._set_state('FINISHED')
                    self.get_logger().info('FINISHED | End of black line reached, stopping robot.')
                elif self.avoid_completed_once and dt <= 1.0:
                    twist.linear.x = 0.012
                    twist.angular.z = 0.0
                elif dt < 2.0:
                    twist.linear.x = 0.03
                    twist.angular.z = 0.12 * self.last_steer_sign
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.20 * self.last_steer_sign
                if not self.task_finished:
                    self.get_logger().warn('LINE LOST | Reacquire hold')

        # Emergency guard only in follow mode.
        if now < self.emergency_brake_until and self.state == 'FOLLOW_LINE':
            twist.linear.x = min(twist.linear.x, 0.02)

        self.cmd_vel_pub.publish(twist)

        try:
            debug_frame = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            if line_detected:
                cv2.circle(debug_frame, (cx, int(roi_h / 2)), 8, (0, 255, 0), 2)
            cv2.putText(
                debug_frame,
                f'Mode:{self.mode} State:{self.state}',
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )
            cv2.line(debug_frame, (int(roi_w / 2), 0), (int(roi_w / 2), roi_h), (0, 0, 255), 2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, 'bgr8'))
        except Exception as exc:
            self.get_logger().error(f'Error publishing debug image: {exc}')

    def laser_callback(self, msg):
        obstacle_threshold = float(self.get_parameter('obstacle_threshold').value)
        min_line_follow_time = float(self.get_parameter('min_line_follow_time_before_avoid').value)
        now = time.time()

        if now - self.start_time < 2.5:
            self.obstacle_hit_count = 0
            return

        if self.line_follow_started_at is None or (now - self.line_follow_started_at) < min_line_follow_time:
            self.obstacle_hit_count = 0
            return

        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isinf(ranges) | np.isnan(ranges)] = np.inf
        ranges[ranges < 0.10] = np.inf

        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        front_mask = np.abs(angles) <= 0.55
        front_slice = ranges[front_mask]

        if front_slice.size == 0:
            front_min = float('inf')
        else:
            front_min = float(np.min(front_slice))
        self.front_distance = front_min

        if front_min < 0.38:
            self.emergency_brake_until = now + 0.45

        if front_min < obstacle_threshold:
            self.obstacle_hit_count += 1
        else:
            self.obstacle_hit_count = max(0, self.obstacle_hit_count - 1)

        if (
            self.obstacle_hit_count >= 4
            and self.state == 'FOLLOW_LINE'
            and now > self.obstacle_cooldown_until
            and now > self.avoid_rearm_until
            and not self.avoid_completed_once
        ):
            self.mode = 'OBSTACLE_AVOID'
            self._set_state('AVOID_STOP')
            self.obstacle_cooldown_until = now + 4.0
            self.obstacle_hit_count = 0
            self.get_logger().warn(
                f'OBSTACLE DETECTED | dist:{front_min:.3f}m threshold:{obstacle_threshold:.2f}m'
            )


def main():
    rclpy.init()
    node = LineFollowerController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
