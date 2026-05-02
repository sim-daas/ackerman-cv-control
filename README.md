# Ackerman Visual Servoing System

An autonomous perception-to-action pipeline for an Ackerman-steered robot in ROS 2 Jazzy and Gazebo Harmonic. The robot locates a specific red target box among multiple decoys and performs precise visual servoing to reach it.

## 🚀 System Architecture

### 1. Perception Node (`perception_node.py`)
- **Isolation Technique**: Uses **Saturation-based thresholding** (HSV S-channel) to separate colorful objects from the gray simulation environment. This effectively ignores the horizon line.
- **Geometry Logic**: Identifies quadrilaterals using polygon approximation.
- **Validation**: Implements a "Squareness Check" that validates edge length consistency and aspect ratio to distinguish the target cube from rectangular decoys like the cylinder.

### 2. Control Node (`control_node.py`)
- **Dual-PID Controller**: One PID loop for steering (lateral error) and one for speed (area/distance error).
- **Non-Holonomic Constraints**: Enforces a minimum forward velocity during steering to satisfy Ackerman kinematics (the robot cannot spin in place).
- **State Machine**:
  - `SEARCHING`: Performs a circular scan pattern if no target is visible.
  - `TRACKING`: Active PID-driven navigation toward the detected box.
  - `ARRIVED`: Stops and stabilizes once the target reaches a specific size in the FOV.

### 3. Simulation & Config
- **Controller**: Uses the modern `ackermann_steering_controller` from `ros2_controllers`.
- **URDF**: A custom Xacro model with independent steering joints and a `gz_ros2_control` hardware interface.
- **Randomizer**: Includes a Python utility to randomize shape positions while enforcing a 50-degree angular separation to prevent occlusions.

## 🛠 Usage

### Prerequisites
Ensure you have the following ROS 2 Jazzy packages installed:
```bash
sudo apt install ros-jazzy-ackermann-steering-controller ros-jazzy-ros2-controllers ros-jazzy-cv-bridge
```

### Running the System
1. **Randomize the world** (Optional):
   ```bash
   python3 mobilerobo/scripts/randomize_shapes.py
   ```
2. **Launch Simulation**:
   ```bash
   ros2 launch mobilerobo sim.launch.py
   ```
3. **Launch Autonomous Logic**:
   ```bash
   ros2 launch mobilerobo perception_control.launch.py
   ```

### Debugging
View the composite perception feed (Original vs. Mask):
```bash
ros2 run rqt_image_view rqt_image_view /perception/debug_image
```

## 📝 Configuration Highlights
- **Topic**: Commands are sent to `/ackermann_steering_controller/reference`.
- **Sync**: All nodes must have `use_sim_time: true` to synchronize with Gazebo's clock.
- **FOV**: Camera FOV is set to 80° (1.396 rad) for an optimal balance between search capability and edge accuracy.
