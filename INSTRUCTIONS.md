# Supermarket Simulation Guidelines

Here are the commands and steps to perform the assignment tasks using the custom `supermarket.world`.

## 0. Environment Setup (Docker)
Before running the ROS2 commands, ensure your Docker environment is running:

```bash
git clone https://github.com/sim-daas/robobase
cd robobase
xhost +
./docker-run.sh
```

*Once inside the container, you can proceed with the remaining steps.*

## 1. Mapping Simulation (Building the Map)
To build a map of the new supermarket environment, we will use Cartographer as the primary mapping tool.

1. Launch the simulation with the new world:
   ```bash
   ros2 launch mobilerobo sim.launch.py world:=supermarket.world
   ```

2. In a new terminal, launch the Cartographer mapping node:
   ```bash
   ros2 launch mobilerobo slam.launch.py use_sim_time:=true
   ```

3. Drive the robot around the custom gaps and racks using teleop to generate the map:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --param stamped:=true --remap cmd_vel:=/diff_drive_controller/cmd_vel
   ```

4. **Saving the map**:
   Once the map looks good in RViz, save it:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f /root/robows/src/robobase/mobilerobo/maps/supermarket_map
   ```
   *(This will create `supermarket_map.yaml` and `supermarket_map.pgm`)*

## 2. Navigation Simulation
Once the map is saved, configure Nav2 to use it and run the custom auto-navigation script.

1. **Update Map for Nav2**: Ensure the default map in your `nav.launch.py` points to the freshly created `supermarket_map.yaml`. Open `mobilerobo/launch/nav.launch.py` and verify:
   ```python
   map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(mobilerobo_pkg_dir, 'maps', 'supermarket_map.yaml'),
        description='Full path to map file to load'
    )
   ```

2. Run the navigation launch file:
   ```bash
   ros2 launch mobilerobo nav.launch.py world:=supermarket.world
   ```

3. **Autonomous Demonstration**:
   In another terminal, ensure your workspace is built, then run the `auto_navigate` node:
   ```bash
   # Make sure you are in your ROS2 workspace root
   colcon build --packages-select mobilerobo
   source install/setup.bash
   
   ros2 run mobilerobo auto_navigate
   ```
   *The robot will automatically navigate from its start position, stop at a rack, wait 5 seconds, and return to the start.*

## 3. Future Testing & Algorithm Comparison
While slam_toolbox provides a robust mapping solution, the `cartographer` mapping package (available in `cartographer.launch.py`) will also be tested in the future to fulfill comparison requirements. 
By comparing maps generated from both techniques, we can measure and showcase system robustness against minor environmental changes (e.g., placing obstacles in entrances).
