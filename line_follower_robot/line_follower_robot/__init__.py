"""
Line Follower Robot - ROS 2 Autonomous Navigation Package

This package implements a complete autonomous mobile robot system with:
- Vision-based line tracking using OpenCV (HSV color detection)
- LiDAR-based obstacle detection and avoidance
- Intelligent sensor fusion for mode switching
- Real-time robot control in Gazebo simulation

Modules:
    controller: Main sensor fusion and mode switching logic
    line_detector: OpenCV-based vision processing for line tracking
    obstacle_detector: Laser-based obstacle detection and safety

Author: Sathwik
Version: 1.0.0
"""

__version__ = "1.0.0"
__author__ = "Sathwik"
