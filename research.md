# Research & Technical Implementation Notes

This document captures the critical technical challenges and solutions discovered during the implementation of the Ackerman Visual Servoing system on ROS 2 Jazzy and Gazebo Harmonic.

## 1. ROS 2 Jazzy Controller Migration
Migrating to `ackermann_steering_controller` in Jazzy revealed several undocumented or deprecated features:

- **Command Topic**: Unlike previous versions that defaulted to `cmd_vel`, the Jazzy implementation typically expects inputs on the `~/reference` topic.
- **Parameter Renaming**: Many parameter keys were updated for consistency:
  - `rear_wheels_names` → `traction_joints_names`
  - `front_steer_joint_names` → `steering_joints_names`
  - `rear_wheels_state_names` → `traction_joints_state_names`
- **Traction/Steering radius**: Separate parameters are now required for `traction_wheels_radius` and `steering_wheels_radius`.

## 2. Clock Synchronization (`use_sim_time`)
A primary blocker was "silent" command rejection. The controller manager in Gazebo runs on simulation time. If external nodes (Perception/Control) publish messages with Wall Time (Unix Epoch), the controller rejects them because the timestamps are "in the future" (relative to the sim clock) or too old.
- **Solution**: Explicitly setting `use_sim_time: true` in the `controllers.yaml` and passing it as a parameter to every Python node.

## 3. Perception Robustness
### The "Horizon" Problem
Standard Canny edge detection in a simulation environment often picks up the horizon line (where the floor meets the background). If objects are on this line, their contours merge, making them impossible to detect via shape analysis.
- **Solution**: Switched from intensity-based edge detection to **Saturation-based thresholding**. Since the floor and walls are gray (low saturation), but the targets are colorful (high saturation), the horizon line is naturally eliminated, leaving clean object silhouettes.

### Discriminating Cubes from Cylinders
From a distance, a cylinder and a cube both project rectangular 2D contours.
- **Solution**: Implemented a **Squareness Check**.
  - A cube projection is equilateral ($1:1$ ratio).
  - A cylinder projection is elongated ($1.5:1$ ratio).
  - The system now calculates all four edge lengths and the bounding box aspect ratio to reject non-square quadrilaterals.

## 4. Kinematic Constraints
Visual servoing for an Ackerman robot differs significantly from a differential drive (unicycle) model.
- **Problem**: The robot cannot rotate while stationary. A pure angular error command results in zero movement if linear velocity is zero.
- **Solution**: The `control_node` enforces a `min_linear_speed` whenever a steering correction is required, ensuring the wheels can physically rotate to the commanded angle.

## 5. Camera Optics
- **FOV Optimization**: Increasing the horizontal FOV to **80 degrees** proved to be the "sweet spot." It allows the robot to find the target with only a partial rotation while maintaining enough pixel density for accurate edge detection at 5 meters.
