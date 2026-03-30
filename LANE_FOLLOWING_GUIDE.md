# Lane Following System - Complete Setup & Operation Guide

## Overview

This is a ROS 2 lane-following system for autonomous vehicles using Gazebo simulation. The system detects road lanes from camera images and controls steering to keep the vehicle centered on the lane.

**Key Components:**
- **Perception**: Lane detection from camera images (lane_detector_node)
- **Control**: PID-based steering controller (lane_controller)
- **Simulation**: Gazebo with Prius model and compact city world

---

## System Architecture

```
Gazebo Simulation (compact_city.world)
    ↓
Camera: /prius/front_camera/image_raw (23 Hz, RGB8)
    ↓
Lane Detector Node
    ├─ Input: Camera images
    ├─ Algorithm: Grayscale → Histogram Equalization → Canny Edge → ROI Trapezoid → HoughLinesP → Line Fitting
    └─ Output: /lane/error (lateral_error, heading_error, lane_valid)
    ↓
Lane Controller Node
    ├─ Input: /lane/error
    ├─ Algorithm: Low-pass filter → PID steering control
    └─ Output: /prius/cmd_vel (linear speed, angular steering)
    ↓
Prius Model (in Gazebo)
    └─ Executes movement commands
```

---

## Quick Start (3 Steps)

### Step 1: Start Gazebo Simulation

```bash
cd /home/ranim/autoCar_ws
source install/setup.bash
bash scripts/run_compact_city.sh
```

**Wait 5-10 seconds** for Gazebo to fully load. You should see:
- Gazebo GUI window with Prius model on road
- ROS nodes: `/gazebo`, `/prius/*` plugins, `/simple_localizer`
- Topics: `/clock`, `/prius/front_camera/image_raw`, `/prius/odom`, etc.

**Verification:**
```bash
# In another terminal
source ~/autoCar_ws/install/setup.bash
ros2 topic hz /clock                           # Should show ~8 Hz
ros2 topic hz /prius/front_camera/image_raw    # Should show ~23 Hz
```

---

### Step 2: Start Lane Detector

```bash
# Option A: Without debug visualization (headless)
source ~/autoCar_ws/install/setup.bash
ros2 run perception lane_detector_node

# Option B: With debug visualization (if X11/display available)
source ~/autoCar_ws/install/setup.bash
ros2 run perception lane_detector_node --ros-args -p show_debug:=true
```

**Expected Output:**
```
[INFO] [lane_detector]: lane_detector started, image_topic=/prius/front_camera/image_raw
```

**Verification (new terminal):**
```bash
source ~/autoCar_ws/install/setup.bash
timeout 3 ros2 topic echo /lane/error
```

Should show continuous messages like:
```
x: -0.45      # lateral error (negative = lane on left)
y: 0.05       # heading error
z: 1.0        # lane detected (1.0 = valid, 0.0 = fallback)
```

---

### Step 3: Start Lane Controller

```bash
source ~/autoCar_ws/install/setup.bash
ros2 launch control lane_control.launch.py
```

**Expected Output:**
```
[INFO] [lane_controller]: lane_controller started, cmd_topic=/prius/cmd_vel
```

**Verification (new terminal):**
```bash
source ~/autoCar_ws/install/setup.bash
timeout 3 ros2 topic echo /prius/cmd_vel
```

Should show continuous steering commands like:
```
linear.x: 0.45        # forward speed (m/s)
angular.z: -0.18      # steering (rad/s, negative = turn left)
```

**In Gazebo window:** Vehicle should now drive and steer to follow the road lane!

---

## Understanding the System

### Lane Detector Algorithm

1. **Image Preprocessing:**
   - Convert RGB to grayscale
   - Apply histogram equalization for better contrast in dim lighting
   - Gaussian blur to reduce noise

2. **Edge Detection:**
   - Canny edge detection (thresholds: 20-60)
   - Creates binary edge map

3. **Region of Interest (ROI):**
   - Trapezoid ROI: covers lane region ahead of vehicle
   - Bottom: 90% image width
   - Top: 40-60% image width (20% from edges)

4. **Line Detection:**
   - HoughLinesP: finds line segments
   - Slope filtering: |slope| between 0.25-10.0 (prevents near-horizontal/vertical lines)
   - Length filtering: minimum 18 pixels

5. **Lane Fitting:**
   - Separate left (negative slope) and right (positive slope) lines
   - Weighted least-squares fit
   - Fallback: if one side missing, infer from nominal lane width (42% of image width)

6. **Error Calculation:**
   - **Lateral Error** = (lane_center - image_center) / image_center [-1, 1]
   - **Heading Error** = atan2(dx, dy) of center line [-0.8, 0.8 rad]

### Lane Controller Algorithm

1. **Input Processing:**
   - Receive /lane/error (lateral, heading, valid_flag)
   - Low-pass filter: `filtered = (1-α) × filtered + α × raw` where α=0.15

2. **PID Steering Control:**
   - Input: lateral error
   - PID gains: Kp=0.4, Ki=0.005, Kd=0.02
   - Output: steering command [-max_angular, +max_angular]

3. **Heading Correction:**
   - Add heading error × heading_gain (0.2)
   - Keeps vehicle angle aligned with lane

4. **Speed Control:**
   - Base speed: 0.6 m/s
   - Slow down when off-lane: `speed = base × max(0.2, 1 - |lateral_error|)`

5. **Lane Loss Handling:**
   - When lane not detected: exponential speed decay (0.8x per cycle)
   - Steering zeroed out
   - Vehicle gradually stops instead of jerky halt

---

## Parameter Tuning

### Adjust Steering Responsiveness

Edit `/home/ranim/autoCar_ws/src/control/launch/lane_control.launch.py`:

```python
'kp': 0.4,              # Proportional gain (increase for faster response, decrease for stability)
'kd': 0.02,             # Derivative gain (increase to reduce overshoot)
'heading_gain': 0.2,    # Heading error weight (increase to correct angle faster)
'error_alpha': 0.15,    # Low-pass filter (decrease for smoother, less responsive)
```

**If vehicle oscillates (sallanıyor):**
- Decrease `kp` to 0.3 or lower
- Decrease `heading_gain` to 0.1
- Increase `error_alpha` to 0.2-0.25

**If vehicle doesn't steer enough:**
- Increase `kp` to 0.5-0.6
- Increase `heading_gain` to 0.3

**Rebuild after changes:**
```bash
cd /home/ranim/autoCar_ws
colcon build --packages-select control
```

### Adjust Speed

Edit `/home/ranim/autoCar_ws/src/control/launch/lane_control.launch.py`:

```python
'target_speed': 0.6,    # Forward speed in m/s (try 0.5-1.0)
```

### Lane Detection Tuning

If lanes not detected, edit `/home/ranim/autoCar_ws/src/perception/src/lane_detector.cpp`:

**Canny thresholds (line ~55):**
```cpp
cv::Canny(blur, edges, 20.0, 60.0);  // Lower = more sensitive, but noisier
```

Try: `(15, 45)` for dim images, `(30, 90)` for bright images

**HoughLinesP thresholds (line ~81):**
```cpp
cv::HoughLinesP(roi_edges, lines, 1.0, CV_PI / 180.0, 15, 18.0, 12.0);
//                                                      ^^  ^^^   ^^^
//                                           threshold  minLen  maxGap
```

Decrease threshold for more line detection, increase for fewer false positives.

---

## Troubleshooting

### Camera shows no image or very dim image
- **Cause:** Gazebo lighting or camera angle
- **Solution:** Check world file at `src/worlds/compact_city.world`
- **Workaround:** Histogram equalization already applied in detector

### Lane not detected (fallback warnings)
```
[WARN] Lane not detected. Publishing zero error fallback.
```

- **Diagnosis:** Run detector with debug:
  ```bash
  ros2 run perception lane_detector_node --ros-args -p show_debug:=true
  ```
  Check if lane lines are visible in debug window

- **Fix Option 1:** Lower Canny thresholds in `lane_detector.cpp` (see tuning section)
- **Fix Option 2:** Check if vehicle is on road in Gazebo
- **Fix Option 3:** Adjust ROI trapezoid region (lines 58-65 in `lane_detector.cpp`)

### Vehicle oscillates or sallanıyor
- **Cause:** Steering gains too high
- **Solution:** Reduce `kp` and `heading_gain` (see Parameter Tuning section)
- **Quick test:**
  ```bash
  ros2 launch control lane_control.launch.py --ros-args -p kp:=0.25 -p heading_gain:=0.1
  ```

### Vehicle doesn't steer
- **Cause 1:** `/lane/error` topic not flowing
  ```bash
  ros2 topic echo /lane/error  # Should show continuous data
  ```
- **Cause 2:** Lane controller crashed
  ```bash
  ps aux | grep lane_controller  # Check if process running
  ```
- **Cause 3:** Wrong default topic for camera
  - Verify: `ros2 topic hz /prius/front_camera/image_raw` shows ~23 Hz
  - If not, check node code for hardcoded wrong topic

### Vehicle reverses or goes sideways
- **Cause:** Steering sign inverted
- **Solution:** Add `--ros-args -p steer_sign:=-1.0` to controller launch
  ```bash
  ros2 launch control lane_control.launch.py --ros-args -p steer_sign:=-1.0
  ```

---

## Complete Working Example (Copy-Paste Ready)

**Terminal 1: Start Gazebo**
```bash
cd /home/ranim/autoCar_ws && source install/setup.bash && bash scripts/run_compact_city.sh
```

**Terminal 2: Start Lane Detector (with debug)**
```bash
source /home/ranim/autoCar_ws/install/setup.bash
ros2 run perception lane_detector_node --ros-args -p show_debug:=true
```

**Terminal 3: Start Controller**
```bash
source /home/ranim/autoCar_ws/install/setup.bash
ros2 launch control lane_control.launch.py
```

**Terminal 4: Monitor System**
```bash
source /home/ranim/autoCar_ws/install/setup.bash

# Watch lane errors
echo "=== Lane Error ===" && ros2 topic echo /lane/error | head -20

# Watch steering commands
echo "=== Steering Command ===" && ros2 topic echo /prius/cmd_vel | head -20

# Check system health
echo "=== Active Nodes ===" && ros2 node list
echo "=== Active Topics ===" && ros2 topic list | grep -E "(lane|cmd_vel)"
```

---

## Advanced: Building & Testing

### Rebuild All Packages

```bash
cd /home/ranim/autoCar_ws
colcon build --packages-select perception control
```

### Run Unit Tests (if available)

```bash
colcon test --packages-select perception control
```

### View Build Artifacts

```bash
ls -lh install/perception/lib/perception/
ls -lh install/control/lib/control/
```

---

## File Structure

```
autoCar_ws/
├── src/
│   ├── perception/
│   │   ├── src/
│   │   │   ├── lane_detector.cpp       # Core algorithm
│   │   │   ├── lane_detector_node.cpp  # ROS wrapper
│   │   │   └── CMakeLists.txt
│   │   └── package.xml
│   └── control/
│       ├── src/
│       │   ├── lane_controller.cpp     # PID controller
│       │   └── CMakeLists.txt
│       ├── launch/
│       │   └── lane_control.launch.py  # Launch config with defaults
│       └── package.xml
├── scripts/
│   └── run_compact_city.sh             # Gazebo launcher
├── config/
│   └── *.rviz                          # Visualization configs
└── LANE_FOLLOWING_GUIDE.md             # This file
```

---

## ROS 2 Topics Reference

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/prius/front_camera/image_raw` | Source | `sensor_msgs::Image` | 23 Hz camera feed (RGB8) |
| `/lane/error` | Lane Detector → Controller | `geometry_msgs::Vector3` | x=lateral, y=heading, z=valid |
| `/prius/cmd_vel` | Controller → Gazebo | `geometry_msgs::Twist` | linear.x=speed, angular.z=steering |
| `/clock` | Gazebo | `rosgraph_msgs::Clock` | Simulation clock (~8 Hz) |
| `/prius/odom` | Gazebo | `nav_msgs::Odometry` | Odometry data |

---

## FAQ

**Q: Can I use this on real hardware?**
A: This is currently Gazebo-only. For real vehicle: replace camera source, adjust parameters for real-world lighting, test safety features.

**Q: How fast does the system run?**
A: Lane detector processes at camera rate (~23 Hz), controller runs at ~20 Hz. Total latency ~50-100 ms.

**Q: Can I add waypoint following?**
A: Yes! The `lane/error` output can be combined with waypoint navigation. See `PATH_PLANNING_GUID.md`.

**Q: How do I record the vehicle's path?**
A: Use ROS 2 bag recording:
```bash
ros2 bag record /prius/odom /lane/error /prius/cmd_vel
```

**Q: Can I tune parameters while running?**
A: Yes, use dynamic reconfigure or restart with `--ros-args -p param:=value`.

---

## Support & Debugging

For detailed algorithm info, see:
- Lane detection: [src/perception/src/lane_detector.cpp](src/perception/src/lane_detector.cpp)
- Control loop: [src/control/src/lane_controller.cpp](src/control/src/lane_controller.cpp)

Common commands:
```bash
# Kill all lane nodes
pkill -f lane_detector
pkill -f lane_controller

# View logs
tail -20 ~/.ros/log/latest/lane_*

# Monitor resource usage
watch -n 0.5 'ps aux | grep lane'

# Test image processing offline (if debug mode enabled)
# Debug window shows: camera image + detected lanes (green=center, colors=edges)
```

---

## Version History

- **v1.0 (2026-03-24):** Initial working system
  - Lane detection with Canny edge + HoughLinesP
  - PID steering controller
  - Low-pass filtering for smooth control
  - Gazebo integration with compact_city world
  - Debug visualization with OpenCV windows (X11 required)

---

**Last Updated:** 2026-03-24  
**Status:** ✅ Fully Operational
