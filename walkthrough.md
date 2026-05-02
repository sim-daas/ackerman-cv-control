# Supermarket Simulation Walkthrough

## Summary of Accomplishments
1. **Gazebo World Creation**: Developed `supermarket.world` featuring a 15x15m layout, 2 entrances/exits, 3 large storage racks in the center, and 3 cylindrical/cuboid static obstacles. Validated that `mini-robo` can smoothly navigate through the gaps.
2. **Launch System Enhancement**: Parameterized the spawn coordinates across `sim.launch.py`, `nav.launch.py`, `slam.launch.py`, and `cartographer.launch.py` to correctly initialize the map origin versus the robot spawn location, resulting in a consistent tf tree and avoiding "Robot out of bounds" errors.
3. **Mapping Implementation (Cartographer)**: Successfully ran the custom `cartographer.launch.py` and `cartographer.lua` configuration to navigate the environment, build a highly accurate grid representation of the supermarket layout, and save the resulting `.yaml` and `.pgm` files.
4. **Autonomous Navigation**: Wrote and executed `auto_navigate.py`, which acts as a Simple Action Client. The script dispatched Navigation2 goal poses to command the robot to automatically move from the entrance to one of the racks, wait for 5 seconds, and seamlessly return to the entry point without manual RViz interaction.

## Future Milestones
- **SLAM Toolbox Validation**: The `slam.launch.py` configuration utilizing SLAM Toolbox remains available in the codebase. As part of future milestones, mapping will be re-run with this algorithm to perform a detailed comparison against Cartographer's output. This will evaluate mapping precision and system robustness to minor environmental variations.
