# Line Follower Robot - Quick Start Guide

## 🚀 Quick Start (30 seconds)

```bash
# 1. Navigate to workspace
cd ~/line_follow_ws

# 2. Build the project
colcon build --symlink-install

# 3. Source the setup
source install/setup.bash

# 4. Run everything with one command
ros2 launch line_follower_robot sim.launch.py
```

**That's it!** Your robot will automatically:
1. ✅ Spawn in Gazebo with a black line
2. ✅ Start tracking the line with vision
3. ✅ Detect and avoid obstacles with LiDAR
4. ✅ Display everything in RViz in real-time

---

## 🎯 What Happens When You Run It

### Wait Times After Launch
- **0-5 seconds**: Gazebo loads the world
- **5-8 seconds**: Robot spawns into simulation
- **8+ seconds**: Controller starts, robot begins autonomous navigation

### Expected Behavior
1. Robot uses camera to find black line
2. Drives forward while keeping line centered
3. When obstacle is detected (< 0.6m), robot stops
4. Robot rotates to find clear path
5. Resumes line following

---

## 📊 Monitor Robot in Real-Time

Open a new terminal and use these commands:

```bash
# See all ROS 2 nodes running
ros2 node list

# See all topics being published
ros2 topic list

# Check camera feed rate
ros2 topic hz /camera/image_raw

# Check laser scan rate  
ros2 topic hz /scan

# See velocity commands being sent
ros2 topic echo /cmd_vel

# Check robot odometry
ros2 topic echo /odom
```

---

## 🎮 Manual Control (For Testing)

```bash
# Send velocity command manually
ros2 topic pub /cmd_vel geometry_msgs/Twist "
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5
" -1
```

---

## 🔧 Configure Parameters

Edit controller behavior by modifying launch file:

**File**: `~/line_follow_ws/src/line_follower_robot/launch/sim.launch.py`

```python
parameters=[{
    'obstacle_threshold': 0.6,      # Detection distance (meters)
    'line_threshold': 100,          # Min pixels to detect line
    'max_linear_speed': 0.3,        # Forward speed (m/s)
    'max_angular_speed': 1.0,       # Max turning speed (rad/s)
}]
```

Then rebuild and relaunch:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch line_follower_robot sim.launch.py
```

---

## 🎥 View Camera Feed

In RViz, the camera image should be visible in the left panel. If not:

1. Click "Add" button in RViz
2. Select "Image"
3. Choose topic: `/camera/image_raw`

---

## 📁 Important Files

| File | Purpose |
|------|---------|
| `controller.py` | Main sensor fusion & mode switching |
| `line_detector.py` | Vision-based line tracking |
| `obstacle_detector.py` | LiDAR-based safety |
| `robot.urdf` | Robot model with sensors |
| `line_track.world` | Gazebo simulation world |
| `robot_view.rviz` | RViz visualization setup |

---

## ❓ FAQs

**Q: Robot not moving?**  
A: Check that controller node started. Wait 8+ seconds after launch.

**Q: No camera image in RViz?**  
A: Add Image display and select `/camera/image_raw` topic.

**Q: Line not being detected?**  
A: Check that robot is over the black line in Gazebo.

**Q: Obstacle avoidance not working?**  
A: Verify laser scan data with `ros2 topic echo /scan`.

---

## 📞 Support

All code is documented with comments. For detailed explanation, see:
- `PROJECT_DOCUMENTATION.md` - Complete system documentation
- Code comments in each Python file

---

**Status**: ✅ Ready to use  
**Last Updated**: March 2026
