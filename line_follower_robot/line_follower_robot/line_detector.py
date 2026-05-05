#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')

        self.bridge = CvBridge()

        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.error_pub = self.create_publisher(Float32, '/line_error', 10)

        self.get_logger().info("Line Detector Started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        h, w = frame.shape[:2]
        crop = frame[int(h/2):h, :]

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])

        mask = cv2.inRange(hsv, lower_black, upper_black)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        M = cv2.moments(mask)

        msg_out = Float32()

        if M["m00"] > 200:
            cx = int(M["m10"] / M["m00"])
            error = cx - w/2
            msg_out.data = float(error)
        else:
            msg_out.data = 9999.0  # line lost

        self.error_pub.publish(msg_out)


def main():
    rclpy.init()
    node = LineDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
