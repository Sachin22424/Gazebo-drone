#!/bin/bash

# Quick start script for multi-drone simulation

echo "========================================"
echo "Multi-Drone Simulation - Quick Start"
echo "========================================"
echo ""

# Source the workspace
source /home/sachin/catkin_ws/devel/setup.bash

echo "Starting simulation..."
echo "- 8 drones will spawn at the starting position"
echo "- They will automatically navigate to 6 colored areas"
echo "- Watch them in Gazebo!"
echo ""
echo "Press Ctrl+C to stop the simulation"
echo ""

# Launch the simulation
roslaunch multi_drone_sim multi_drone_sim.launch
