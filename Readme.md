# Robobase Simulation 

This repository contains the simulation environment and configuration for the mobile robotics assignment.

## Getting Started

To launch the environment using Docker:

1. Clone the repository:
   ```bash
   git clone https://github.com/sim-daas/robobase
   cd robobase
   ```

2. Enable X server access (required for Gazebo/RViz GUI):
   ```bash
   xhost +
   ```

3. Run the Docker container:
   ```bash
   ./docker-run.sh
   ```

## Instructions

For detailed steps on how to execute the Mapping and Navigation tasks inside the Docker container, please refer to [INSTRUCTIONS.md](INSTRUCTIONS.md).
